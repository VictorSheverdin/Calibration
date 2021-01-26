#include "super_glue_matcher.hpp"


namespace marker
{
    namespace tch_u = libtorch_utils;
    namespace trt_u = tensor_rt_utils;

    SuperGlueMatcher::SuperGlueMatcher(const std::string& engine_path, trt_u::Logger& logger)
        : m_super_glue(trt_u::load_engine(engine_path, logger), logger)
    {
        m_output.scores = tch_u::make_tensor_2d(trt_u::Dims2d(0, 0), torch::kFloat32, torch::kCUDA);
    }

    void SuperGlueMatcher::match(const KeypointSetArray& src)
    {
        bool keypoint_count_ok = true;
        for (int i = 0; i < IMAGE_COUNT; ++i)
            keypoint_count_ok = keypoint_count_ok && m_super_glue.resize_bindings(i, src[i].keypoints.size(0));
        
        if (keypoint_count_ok)
        {
            //  Copy keypoint counts to device
            auto keypoint_counts = tch_u::make_tensor_2d(m_super_glue.keypoint_count_dims(), torch::kFloat32, torch::kCPU);
            auto accessor = keypoint_counts.accessor<float, 2>();
            for(int i = 0; i < IMAGE_COUNT; ++i)
                accessor[0][i] = src[i].keypoints.size(0);
            keypoint_counts = keypoint_counts.to(torch::kCUDA);

            //  Prepare output scores buffer
            m_output.scores.resize_({ 1, src[0].keypoints.size(0) + 1, src[1].keypoints.size(0) + 1 });

            //  Set buffer pointers and do forward
            std::vector<void*> superglue_buffers;
            for(const KeypointSet& ks : src )
                superglue_buffers.push_back(ks.descriptors.data_ptr());
            for(const KeypointSet& ks : src )
                superglue_buffers.push_back(ks.keypoints.data_ptr());
            for(const KeypointSet& ks : src )
                superglue_buffers.push_back(ks.scores.data_ptr());
            superglue_buffers.push_back(keypoint_counts.data_ptr());
            superglue_buffers.push_back(m_output.scores.data_ptr());
            m_super_glue.execute(superglue_buffers.data());
        }
        else
            m_output.scores.resize_({ 1, 0, 0 });
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

    void SuperGlueMatcher::performance_test_match(
        const KeypointSetArray& src, PerformanceStats& perf_stats)
    {
        namespace chr = std::chrono;
        using Clock = chr::steady_clock;

        static auto diff_time = [](Clock::time_point beg, Clock::time_point end) -> chr::microseconds
        {
            return chr::duration_cast<chr::microseconds>(end - beg);
        };

        auto on_resize_bindings = chr::steady_clock::now();

        bool keypoint_count_ok = true;
        for (int i = 0; i < IMAGE_COUNT; ++i)
            keypoint_count_ok = keypoint_count_ok && m_super_glue.resize_bindings(i, src[i].keypoints.size(0));
        
        if (keypoint_count_ok)
        {
            auto on_keypoint_count = chr::steady_clock::now();

            //  Copy keypoint counts to device
            auto keypoint_counts = tch_u::make_tensor_2d(m_super_glue.keypoint_count_dims(), torch::kFloat32, torch::kCPU);
            auto accessor = keypoint_counts.accessor<float, 2>();
            for(int i = 0; i < IMAGE_COUNT; ++i)
                accessor[0][i] = src[i].keypoints.size(0);
            keypoint_counts = keypoint_counts.to(torch::kCUDA);

            //  Prepare output scores buffer
            m_output.scores.resize_({ 1, src[0].keypoints.size(0) + 1, src[1].keypoints.size(0) + 1 });

            auto on_forward = chr::steady_clock::now();

            //  Set buffer pointers and do forward
            std::vector<void*> superglue_buffers;
            for(const KeypointSet& ks : src )
                superglue_buffers.push_back(ks.descriptors.data_ptr());
            for(const KeypointSet& ks : src )
                superglue_buffers.push_back(ks.keypoints.data_ptr());
            for(const KeypointSet& ks : src )
                superglue_buffers.push_back(ks.scores.data_ptr());
            superglue_buffers.push_back(keypoint_counts.data_ptr());
            superglue_buffers.push_back(m_output.scores.data_ptr());
            m_super_glue.execute(superglue_buffers.data());
            cudaDeviceSynchronize();

            auto on_exit = chr::steady_clock::now();

            if (perf_stats.call_count > 0)
            {
                perf_stats.binding_resize_duration += diff_time(on_resize_bindings, on_keypoint_count);
                perf_stats.keypoint_count_duration += diff_time(on_keypoint_count, on_forward);
                perf_stats.forward_duration += diff_time(on_forward, on_exit);
            }
            ++perf_stats.call_count;
        }
        else
            m_output.scores.resize_({ 1, 0, 0 });
    }
}