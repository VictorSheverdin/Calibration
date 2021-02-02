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
        m_input_mask_resized = cv::cuda::createContinuous(m_scores_size, CV_8UC1);
        m_score_mask = tch_u::make_tensor_4d(scores_dims, torch::kBool, torch::kCUDA);
        m_inner_area = make_inner_area_mask(m_score_mask.sizes(), m_cfg.border, torch::kCUDA);
        m_topk_indices = tch_u::make_tensor_1d(0, torch::kLong, torch::kCUDA);
    }

    void KeypointSelector::select(const ScoreMap& score_map, int count, KeypointSet& dst)
    {
        select(score_map, cv::Mat(), count, dst);
    }

    void KeypointSelector::select(const ScoreMap& score_map, 
        const cv::Mat& mask, int count, KeypointSet& dst)
    {
        
        apply_threshold_mask(score_map);    
        if (!mask.empty())
        {
            upload_mask(mask);
            apply_input_mask();
        }
        select_inner(score_map, count, dst);
    }

    void KeypointSelector::upload_mask(const cv::Mat& mask)
    {
        if (mask.rows != m_scores_size.height || mask.cols != m_scores_size.width)
        {
            if (mask.rows != m_input_mask.rows || mask.cols != m_input_mask.cols)
                m_input_mask.create(mask.rows, mask.cols, mask.type());
            m_input_mask.upload(mask);

            //  Mask is binary
            cv::cuda::resize(m_input_mask, m_input_mask_resized, 
                m_scores_size, 0., 0., cv::INTER_NEAREST);
        }
        else
            m_input_mask_resized.upload(mask);
    }

    void KeypointSelector::apply_input_mask()
    {
        auto input_mask_tensor = tch_u::as_tensor(m_input_mask_resized, torch::kBool);
        m_score_mask.select(0, 0).select(0, 0).logical_and_(input_mask_tensor);
    }

    void KeypointSelector::apply_threshold_mask(const ScoreMap& score_map)
    {
        //  Mask out scores that are too low or too close to border
        torch::gt_out(m_score_mask, score_map.values, m_cfg.score_threshold);
        m_score_mask.logical_and_(m_inner_area);
    }

    void KeypointSelector::select_inner(const ScoreMap& score_map, int count, KeypointSet& dst)
    {
        auto keypoints = m_score_mask.select(0, 0).select(0, 0).nonzero();
        auto scores = score_map.values.select(0, 0).select(0, 0)
            .index({ keypoints.select(1, 0), keypoints.select(1, 1) });
        keypoints = keypoints.fliplr().to(torch::kFloat32);
        if (keypoints.size(0) > count)
        {
            dst.scores.resize_({ count });
            m_topk_indices.resize_({ count });
            torch::topk_out(dst.scores, m_topk_indices, scores, 
                /*k=*/count, /*dim=*/0, /*largest=*/true, /*sorted=*/false);
            
            dst.keypoints.resize_({ count, keypoints.size(1) });
            torch::index_select_out(dst.keypoints, keypoints, 
                /*dim=*/0, /*index=*/m_topk_indices);
        }
        else
        {
            dst.keypoints = keypoints;
            dst.scores = scores;
        }

        // Change keypoint coordinates to match source image size
        float x_ratio = static_cast<float>(score_map.image_size.width) / score_map.values.size(3);
        float y_ratio = static_cast<float>(score_map.image_size.height) / score_map.values.size(2);
        dst.keypoints.select(1, 0).mul_(x_ratio);
        dst.keypoints.select(1, 1).mul_(y_ratio);
        dst.image_size = score_map.image_size;
    }
}