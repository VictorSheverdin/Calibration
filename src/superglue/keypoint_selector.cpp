#include "keypoint_selector.hpp"


namespace marker
{
    namespace tf = torch::nn::functional;
    namespace tch_u = libtorch_utils;
    namespace trt_u = tensor_rt_utils;

    at::Tensor make_inner_area_mask(torch::IntArrayRef shape, std::int64_t border, c10::Device device)
    {
        std::vector<int64_t> inner_shape{ 
            shape[0], 
            shape[1], 
            shape[2] - 2 * border, 
            shape[3] - 2 * border 
        };
        auto opt = torch::TensorOptions()
            .device(device)
            .dtype(torch::kBool);
        auto border_zero = tf::PadFuncOptions({ 
            border, border, 
            border, border 
        }).mode(torch::kConstant).value(0);

        auto inner = torch::ones(inner_shape, opt);
        return tf::pad(inner, border_zero);
    }

    KeypointSelector::KeypointSelector(const Config& cfg, const trt_u::Dims4d& scores_dims)
        : m_cfg(cfg)
    {   
        m_scores_size = cv::Size2i(scores_dims.cols, scores_dims.rows);
        for(auto& in_mask: m_input_mask_resized)
            in_mask = cv::cuda::createContinuous(m_scores_size, CV_8UC1);
        m_score_mask = tch_u::make_tensor_4d(scores_dims, torch::kBool, torch::kCUDA);
        m_inner_area = make_inner_area_mask(m_score_mask.sizes(), m_cfg.border, torch::kCUDA);
        m_topk_indices = tch_u::make_tensor_1d(0, torch::kLong, torch::kCUDA);
    }

    void KeypointSelector::select_keypoints(const ScoreMap& score_map, int count, KeypointSetArray& dst)
    {
        cv::Mat first_mask, second_mask;
        select_keypoints(score_map, first_mask, second_mask, count, dst);
    }

    void KeypointSelector::select_keypoints(const ScoreMap& score_map, const cv::Mat& first_mask, 
        const cv::Mat& second_mask, int count, KeypointSetArray& dst)
    {
        //  Mask out scores that are too low or too close to border
        torch::gt_out(m_score_mask, score_map.values, m_cfg.score_threshold);
        m_score_mask.logical_and_(m_inner_area);

        //  Check if there are input masks
        std::array<bool, IMAGE_COUNT> use_mask;
        use_mask[0] = !first_mask.empty();
        use_mask[1] = !second_mask.empty();
        
        //  Copy masks to device with possible resize and apply them
        if (use_mask[0])
        {
            if (first_mask.rows != m_scores_size.height || first_mask.cols != m_scores_size.width)
            {
                if (first_mask.rows != m_input_mask[0].rows || first_mask.cols != m_input_mask[0].cols)
                    m_input_mask[0].create(first_mask.rows, first_mask.cols, first_mask.type());
                m_input_mask[0].upload(first_mask);
                cv::cuda::resize(m_input_mask[0], m_input_mask_resized[0], m_scores_size);
            }
            else
                m_input_mask_resized[0].upload(first_mask);
        }
        if (use_mask[1])
        {
            if (second_mask.rows != m_scores_size.height || second_mask.cols != m_scores_size.width)
            {
                if (second_mask.rows != m_input_mask[1].rows || second_mask.cols != m_input_mask[1].cols)
                    m_input_mask[1].create(second_mask.rows, second_mask.cols, second_mask.type());
                m_input_mask[1].upload(second_mask);
                cv::cuda::resize(m_input_mask[1], m_input_mask_resized[1], m_scores_size);
            }
            else
                m_input_mask_resized[1].upload(second_mask);
        }

        
        for(int i = 0; i < dst.size(); ++i)
        {
            if(use_mask[i])
            {
                auto input_mask_tensor = tch_u::as_tensor(m_input_mask_resized[i], torch::kBool);
                m_score_mask.select(0, i).select(0, 0).logical_and_(input_mask_tensor);
            }
            auto keypoints = m_score_mask.select(0, i).select(0, 0).nonzero();
            auto scores = score_map.values.select(0, i).select(0, 0)
                .index({ keypoints.select(1, 0), keypoints.select(1, 1) });
            keypoints = keypoints.fliplr().to(torch::kFloat32);
            if (keypoints.size(0) > count)
            {
                dst[i].scores.resize_({ count });
                m_topk_indices.resize_({ count });
                torch::topk_out(dst[i].scores, m_topk_indices, scores, 
                    /*k=*/count, /*dim=*/0, /*largest=*/true, /*sorted=*/false);
                
                dst[i].keypoints.resize_({ count, keypoints.size(1) });
                torch::index_select_out(dst[i].keypoints, keypoints, 
                    /*dim=*/0, /*index=*/m_topk_indices);
            }
            else
            {
                dst[i].keypoints = keypoints;
                dst[i].scores = scores;
            }

            // Change keypoint coordinates to match source image size
            float x_ratio = static_cast<float>(score_map.image_size[i].width) / score_map.values.size(3);
            float y_ratio = static_cast<float>(score_map.image_size[i].height) / score_map.values.size(2);
            dst[i].keypoints.select(1, 0).mul_(x_ratio);
            dst[i].keypoints.select(1, 1).mul_(y_ratio);
            dst[i].image_size = score_map.image_size[i];
        }
    }

    KeypointSetArray KeypointSelector::make_keypoints()
    {
        KeypointSetArray ret;
        for(auto& ks: ret)
        {
            ks.keypoints = tch_u::make_tensor_2d(trt_u::Dims2d(0, 2), torch::kFloat32, torch::kCUDA);
            ks.scores = tch_u::make_tensor_1d(0, torch::kFloat32, torch::kCUDA);
        }
        return ret;
    }
}