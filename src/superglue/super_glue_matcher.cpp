#include "super_glue_matcher.hpp"


namespace marker
{
    namespace tch_u = libtorch_utils;
    namespace trt_u = tensor_rt_utils;


    SuperGlueMatcher::SuperGlueMatcher(const std::string& engine_path, trt_u::Logger& logger)
        : m_super_glue(trt_u::load_engine(engine_path, logger), logger)
    {
        //  Initialize CPU and GPU buffers for keypoint counts and image shapes
        //  Use one buffer for all those parameters to copy memory only once
        auto kcd = m_super_glue.keypoint_count_dims();
        auto lsd = m_super_glue.left_image_shape_dims();
        auto rsd = m_super_glue.right_image_shape_dims();
        m_params_cpu.create(1, kcd.count + lsd.count + rsd.count, CV_32FC1);
        m_params_gpu = cv::cuda::createContinuous(m_params_cpu.rows, m_params_cpu.cols, CV_32FC1);

        //  Initialize output buffer data type and device
        m_output.scores = tch_u::make_tensor_2d(trt_u::Dims2d(0, 0), torch::kFloat32, torch::kCUDA);
    }

    void SuperGlueMatcher::match(const KeypointSet& left_keypoints, const KeypointSet& right_keypoints, 
        const DescriptorSet& left_descriptors, const DescriptorSet& right_descriptors)
    {
        //  Try to resize SuperGlue bindings
        std::int64_t left_count = left_keypoints.count();
        std::int64_t right_count = right_keypoints.count();
        if (!m_super_glue.resize_bindings(0, left_count))
        {
            m_output.scores.resize_({ 1, 0, 0 });
            return;
        }
        if (!m_super_glue.resize_bindings(1, right_count))
        {
            m_output.scores.resize_({ 1, 0, 0 });
            return;
        }

        // Copy keypoint counts and image shapes to device.
        m_params_cpu.at<float>(0, 0) = left_count;
        m_params_cpu.at<float>(0, 1) = right_count;
        m_params_cpu.at<float>(0, 2) = left_keypoints.image_size.height;
        m_params_cpu.at<float>(0, 3) = left_keypoints.image_size.width;
        m_params_cpu.at<float>(0, 4) = right_keypoints.image_size.height;
        m_params_cpu.at<float>(0, 5) = right_keypoints.image_size.width;
        m_params_gpu.upload(m_params_cpu);
        auto kpt_count_ptr = m_params_gpu.cudaPtr();
        auto left_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 2;
        auto right_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 4;

        //  Prepare output scores buffer
        m_output.scores.resize_({ 1, left_count + 1, right_count + 1 });

        //  Set buffer pointers and do forward
        void* superglue_buffers[] = {
            left_img_shape_ptr, right_img_shape_ptr,
            left_descriptors.values.data_ptr(), right_descriptors.values.data_ptr(),
            left_keypoints.keypoints.data_ptr(), right_keypoints.keypoints.data_ptr(),
            left_keypoints.scores.data_ptr(), right_keypoints.scores.data_ptr(),
            kpt_count_ptr,
            m_output.scores.data_ptr()
        };
        m_super_glue.execute(superglue_buffers);        
    }

    int SuperGlueMatcher::keypoint_size() const
    {
        return m_super_glue.keypoints_dims(0).depth;
    }

    int SuperGlueMatcher::score_size() const
    {
        return m_super_glue.scores_dims(0).depth;
    }

    int SuperGlueMatcher::descr_size() const
    {
        return m_super_glue.descr_dims(0).depth;
    }

    const MatchTable& SuperGlueMatcher::output() const
    {
        return m_output;
    }

    //  ------------------------------------------------------------- Performance measurement utilities

    SuperGlueMatcher::PerformanceStats::PerformanceStats()
        : binding_resize_duration(0)
        , keypoint_count_duration(0)
        , forward_duration(0)
        , call_count(0)
    {}

    void SuperGlueMatcher::performance_test_match(const KeypointSet& left_keypoints, const KeypointSet& right_keypoints, 
        const DescriptorSet& left_descriptors, const DescriptorSet& right_descriptors, PerformanceStats& perf_stats)
    {
        namespace chr = std::chrono;
        using Clock = chr::steady_clock;

        static auto diff_time = [](Clock::time_point beg, Clock::time_point end) -> chr::microseconds
        {
            return chr::duration_cast<chr::microseconds>(end - beg);
        };

        auto on_resize_bindings = chr::steady_clock::now();

        //  Try to resize SuperGlue bindings
        std::int64_t left_count = left_keypoints.count();
        std::int64_t right_count = right_keypoints.count();
        if (!m_super_glue.resize_bindings(0, left_count))
        {
            m_output.scores.resize_({ 1, 0, 0 });
            return;
        }
        if (!m_super_glue.resize_bindings(1, right_count))
        {
            m_output.scores.resize_({ 1, 0, 0 });
            return;
        }

        auto on_keypoint_count = chr::steady_clock::now();

        // Copy keypoint counts and image shapes to device.
        m_params_cpu.at<float>(0, 0) = left_count;
        m_params_cpu.at<float>(0, 1) = right_count;
        m_params_cpu.at<float>(0, 2) = left_keypoints.image_size.height;
        m_params_cpu.at<float>(0, 3) = left_keypoints.image_size.width;
        m_params_cpu.at<float>(0, 4) = right_keypoints.image_size.height;
        m_params_cpu.at<float>(0, 5) = right_keypoints.image_size.width;
        m_params_gpu.upload(m_params_cpu);
        auto kpt_count_ptr = m_params_gpu.cudaPtr();
        auto left_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 2;
        auto right_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 4;

        //  Prepare output scores buffer
        m_output.scores.resize_({ 1, left_count + 1, right_count + 1 });

        auto on_forward = chr::steady_clock::now();

        //  Set buffer pointers and do forward
        void* superglue_buffers[] = {
            left_img_shape_ptr, right_img_shape_ptr,
            left_descriptors.values.data_ptr(), right_descriptors.values.data_ptr(),
            left_keypoints.keypoints.data_ptr(), right_keypoints.keypoints.data_ptr(),
            left_keypoints.scores.data_ptr(), right_keypoints.scores.data_ptr(),
            kpt_count_ptr,
            m_output.scores.data_ptr()
        };
        m_super_glue.execute(superglue_buffers);     

        auto on_exit = chr::steady_clock::now();

        if (perf_stats.call_count > 0)
        {
            perf_stats.binding_resize_duration += diff_time(on_resize_bindings, on_keypoint_count);
            perf_stats.keypoint_count_duration += diff_time(on_keypoint_count, on_forward);
            perf_stats.forward_duration += diff_time(on_forward, on_exit);
        }
        ++perf_stats.call_count;   
    }
}