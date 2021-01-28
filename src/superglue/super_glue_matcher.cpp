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

    void SuperGlueMatcher::match(const KeypointSetArray& keypoints, const DescriptorSetArray& descriptors)
    {
        //  Try to resize SuperGlue bindings
        std::array<std::int64_t, IMAGE_COUNT> keypoint_count;
        for (int i = 0; i < IMAGE_COUNT; ++i)
        {
            keypoint_count[i] = keypoints[i].keypoints.size(0);
            if (!m_super_glue.resize_bindings(i, keypoint_count[i]))
            {
                m_output.scores.resize_({ 1, 0, 0 });
                return;
            }
        }

        // Copy keypoint counts and image shapes to device.
        m_params_cpu.at<float>(0, 0) = keypoint_count[0];
        m_params_cpu.at<float>(0, 1) = keypoint_count[1];
        m_params_cpu.at<float>(0, 2) = keypoints[0].image_size.height;
        m_params_cpu.at<float>(0, 3) = keypoints[0].image_size.width;
        m_params_cpu.at<float>(0, 4) = keypoints[1].image_size.height;
        m_params_cpu.at<float>(0, 5) = keypoints[1].image_size.width;
        m_params_gpu.upload(m_params_cpu);
        auto kpt_count_ptr = m_params_gpu.cudaPtr();
        auto left_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 2;
        auto right_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 4;

        //  Prepare output scores buffer
        m_output.scores.resize_({ 1, keypoint_count[0] + 1, keypoint_count[1] + 1 });

        //  Set buffer pointers and do forward
        std::vector<void*> superglue_buffers;
        superglue_buffers.push_back(left_img_shape_ptr);
        superglue_buffers.push_back(right_img_shape_ptr);
        for (const DescriptorSet& ds : descriptors)
            superglue_buffers.push_back(ds.values.data_ptr());
        for (const KeypointSet& ks : keypoints)
            superglue_buffers.push_back(ks.keypoints.data_ptr());
        for (const KeypointSet& ks : keypoints)
            superglue_buffers.push_back(ks.scores.data_ptr());
        superglue_buffers.push_back(kpt_count_ptr);
        superglue_buffers.push_back(m_output.scores.data_ptr());
        m_super_glue.execute(superglue_buffers.data());
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

    void SuperGlueMatcher::performance_test_match(const KeypointSetArray& keypoints, 
        const DescriptorSetArray& descriptors, PerformanceStats& perf_stats)
    {
        namespace chr = std::chrono;
        using Clock = chr::steady_clock;

        static auto diff_time = [](Clock::time_point beg, Clock::time_point end) -> chr::microseconds
        {
            return chr::duration_cast<chr::microseconds>(end - beg);
        };

        auto on_resize_bindings = chr::steady_clock::now();

        //  Try to resize SuperGlue bindings
        std::array<std::int64_t, IMAGE_COUNT> keypoint_count;
        for (int i = 0; i < IMAGE_COUNT; ++i)
        {
            keypoint_count[i] = keypoints[i].keypoints.size(0);
            if (!m_super_glue.resize_bindings(i, keypoint_count[i]))
            {
                m_output.scores.resize_({ 1, 0, 0 });
                return;
            }
        }

        auto on_keypoint_count = chr::steady_clock::now();

        // Copy keypoint counts and image shapes to device.
        m_params_cpu.at<float>(0, 0) = keypoint_count[0];
        m_params_cpu.at<float>(0, 1) = keypoint_count[1];
        m_params_cpu.at<float>(0, 2) = keypoints[0].image_size.height;
        m_params_cpu.at<float>(0, 3) = keypoints[0].image_size.width;
        m_params_cpu.at<float>(0, 4) = keypoints[1].image_size.height;
        m_params_cpu.at<float>(0, 5) = keypoints[1].image_size.width;
        m_params_gpu.upload(m_params_cpu);
        auto kpt_count_ptr = m_params_gpu.cudaPtr();
        auto left_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 2;
        auto right_img_shape_ptr = static_cast<float*>(m_params_gpu.cudaPtr()) + 4;

        //  Prepare output scores buffer
        m_output.scores.resize_({ 1, keypoint_count[0] + 1, keypoint_count[1] + 1 });

        auto on_forward = chr::steady_clock::now();

        //  Set buffer pointers and do forward
        std::vector<void*> superglue_buffers;
        superglue_buffers.push_back(left_img_shape_ptr);
        superglue_buffers.push_back(right_img_shape_ptr);
        for (const DescriptorSet& ds : descriptors)
            superglue_buffers.push_back(ds.values.data_ptr());
        for (const KeypointSet& ks : keypoints)
            superglue_buffers.push_back(ks.keypoints.data_ptr());
        for (const KeypointSet& ks : keypoints)
            superglue_buffers.push_back(ks.scores.data_ptr());
        superglue_buffers.push_back(kpt_count_ptr);
        superglue_buffers.push_back(m_output.scores.data_ptr());
        m_super_glue.execute(superglue_buffers.data());

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