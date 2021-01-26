#include "super_point_detector.hpp"


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

    //  src must be 3d (c, h, w) float32 Tensor
    //  keypoints must be 2d (x, y) float32 Tensor
    at::Tensor bilinear_grid_sample(const at::Tensor& src, 
        const at::Tensor& keypoints, std::int64_t rows, std::int64_t cols)
    {
        static auto bilinear_sample = tf::GridSampleFuncOptions()
            .mode(torch::kBilinear)
            .align_corners(true);
        static auto normalize_l2 = tf::NormalizeFuncOptions()
            .dim(1)
            .p(2);

        auto grid = keypoints.clone();

        // Normalize x to [-1, 1]
        auto scale_x = cols / src.size(2);
        grid.slice(1, 0, 1)
            .sub_(scale_x / 2 - .5f)
            .true_divide_(cols - scale_x / 2 - .5f)
            .mul_(2)
            .sub_(1);

        //  Normalize y to [-1, 1]
        auto scale_y = rows / src.size(1);
        grid.slice(1, 1, 2)
            .sub_(scale_y / 2 - .5f)
            .true_divide_(rows - scale_y / 2 - .5f)
            .mul_(2)
            .sub_(1);

        //  Perform grid sample and return normalized version
        auto src_sampled = tf::grid_sample(src.view({ 1, src.size(0), src.size(1), src.size(2) }), 
            grid.view({1, 1, -1, 2}), bilinear_sample);
        return tf::normalize(src_sampled, normalize_l2).select(0, 0).select(1, 0);
    }



    SuperPointDetector::SuperPointDetector(const std::string& engine_path, const Config& cfg, trt_u::Logger& logger)
        : m_super_point(trt_u::load_engine(engine_path, logger), logger)
        , m_cfg(cfg)
    {
        //  Input buffers init
        auto in_dims = m_super_point.images_dims();
        m_gpu_uint8.create(IMAGE_COUNT, in_dims.rows, in_dims.cols, CV_8UC1);
        m_gpu_float.create(IMAGE_COUNT, in_dims.rows, in_dims.cols, CV_32FC1);
        for(int i = 0; i < IMAGE_COUNT; ++i)
        {
            m_input_masks[i] = tch_u::make_tensor_2d(
                trt_u::Dims2d(in_dims.rows, in_dims.cols), torch::kBool, torch::kCUDA);
        }
        
        //  Output buffers init
        auto score_map_dims = m_super_point.scores_dims();
        auto descr_map_dims = m_super_point.descrs_dims();
        m_score_map = tch_u::make_tensor_4d(score_map_dims, torch::kFloat32, torch::kCUDA);
        m_descr_map = tch_u::make_tensor_4d(descr_map_dims, torch::kFloat32, torch::kCUDA);
        m_score_mask = tch_u::make_tensor_4d(score_map_dims, torch::kBool, torch::kCUDA);
        m_inner_area = make_inner_area_mask(m_score_mask.sizes(), m_cfg.border, torch::kCUDA);
    }

    trt_u::Dims3d SuperPointDetector::input_shape() const
    {
        auto in_dims = m_super_point.images_dims();
        return trt_u::Dims3d(in_dims.chan, in_dims.rows, in_dims.cols);
    }

    void SuperPointDetector::detect(
        const cv::Mat& first, const cv::Mat& first_mask, 
        const cv::Mat& second, const cv::Mat& second_mask)
    {
        //  Check if need masks
        std::array<bool, 2> use_mask;
        use_mask[0] = !first_mask.empty();
        use_mask[1] = !second_mask.empty();
        
        //  Copy to device
        m_gpu_uint8.header[0].upload(first);
        if (use_mask[0])
            tch_u::host_to_device(first_mask, m_input_masks[0]);
        m_gpu_uint8.header[1].upload(second);
        if (use_mask[1])
            tch_u::host_to_device(second_mask, m_input_masks[1]);

        //  Preprocess
        for (int i = 0; i < IMAGE_COUNT; ++i)
            m_gpu_uint8.header[i].convertTo(m_gpu_float.header[i], m_gpu_float.header[i].type(), 1.f / 255);

        //  Forward
        void* buffers[] = { m_gpu_float.content.cudaPtr(), 
            m_score_map.data_ptr(), m_descr_map.data_ptr() };
        m_super_point.execute(buffers);

        //  Apply masks to scores, save keypoints, scores and descriptors
        torch::gt_out(m_score_mask, m_score_map, m_cfg.score_threshold);
        m_score_mask.logical_and_(m_inner_area);
        for(int i = 0; i < IMAGE_COUNT; ++i)
        {
            if (use_mask[i])
                m_score_mask.select(0, i).select(0, 0).logical_and_(m_input_masks[i]);
            m_output[i].keypoints = m_score_mask.select(0, i).select(0, 0).nonzero();
            m_output[i].scores = m_score_map.select(0, i).select(0, 0)
                .index({ m_output[i].keypoints.select(1, 0), m_output[i].keypoints.select(1, 1) });
            m_output[i].keypoints = m_output[i].keypoints.fliplr().to(torch::kFloat32);
            m_output[i].descriptors = bilinear_grid_sample(m_descr_map.select(0, i), 
                m_output[i].keypoints, m_score_map.size(2), m_score_map.size(3));
        }
    }

    const KeypointSetArray& SuperPointDetector::output() const
    {
        return m_output;
    }

    //  ------------------------------------------------------------- Performance measurement utilities

    SuperPointDetector::PerformanceStats::PerformanceStats()
        : preprocessing_duration(0)
        , forward_duration(0)
        , keypoint_selection_duration(0)
        , call_count(0)
    {}

    void SuperPointDetector::performance_test_detect(
        const cv::Mat& first, const cv::Mat& first_mask, 
        const cv::Mat& second, const cv::Mat& second_mask, 
        PerformanceStats& perf_stats)
    {
        namespace chr = std::chrono;
        using Clock = chr::steady_clock;

        static auto diff_time = [](Clock::time_point beg, Clock::time_point end) -> chr::microseconds
        {
            return chr::duration_cast<chr::microseconds>(end - beg);
        };


        auto on_preprocess = chr::steady_clock::now();

        //  Check if need masks
        std::array<bool, 2> use_mask;
        use_mask[0] = !first_mask.empty();
        use_mask[1] = !second_mask.empty();

        //  Copy to device
        m_gpu_uint8.header[0].upload(first);
        if (use_mask[0])
            tch_u::host_to_device(first_mask, m_input_masks[0]);
        m_gpu_uint8.header[1].upload(second);
        if (use_mask[1])
            tch_u::host_to_device(second_mask, m_input_masks[1]);

        //  Preprocess
        for (int i = 0; i < IMAGE_COUNT; ++i)
            m_gpu_uint8.header[i].convertTo(m_gpu_float.header[i], m_gpu_float.header[i].type(), 1.f / 255);

        auto on_forward = chr::steady_clock::now();

        //  Forward
        void* buffers[] = { m_gpu_float.content.cudaPtr(), 
            m_score_map.data_ptr(), m_descr_map.data_ptr() };
        m_super_point.execute(buffers);
        cudaDeviceSynchronize();
        
        auto on_select = chr::steady_clock::now();

        //  Apply masks to scores, save keypoints, scores and descriptors
        torch::gt_out(m_score_mask, m_score_map, m_cfg.score_threshold);
        m_score_mask.logical_and_(m_inner_area);
        for(int i = 0; i < IMAGE_COUNT; ++i)
        {
            if (use_mask[i])
                m_score_mask.select(0, i).select(0, 0).logical_and_(m_input_masks[i]);
            m_output[i].keypoints = m_score_mask.select(0, i).select(0, 0).nonzero();
            m_output[i].scores = m_score_map.select(0, i).select(0, 0)
                .index({ m_output[i].keypoints.select(1, 0), m_output[i].keypoints.select(1, 1) });
            m_output[i].keypoints = m_output[i].keypoints.fliplr().to(torch::kFloat32);
            m_output[i].descriptors = bilinear_grid_sample(m_descr_map.select(0, i), 
                m_output[i].keypoints, m_score_map.size(2), m_score_map.size(3));
        }

        auto on_exit = chr::steady_clock::now();
        if (perf_stats.call_count > 0)
        {
            perf_stats.preprocessing_duration += diff_time(on_preprocess, on_forward);
            perf_stats.forward_duration += diff_time(on_forward, on_select);
            perf_stats.keypoint_selection_duration += diff_time(on_select, on_exit);
        }
        ++perf_stats.call_count;
    }
}