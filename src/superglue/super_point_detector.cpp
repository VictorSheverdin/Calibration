#include "super_point_detector.hpp"


namespace marker
{
    namespace tf = torch::nn::functional;
    namespace tch_u = libtorch_utils;
    namespace trt_u = tensor_rt_utils;


    SuperPointDetector::SuperPointDetector(const std::string& engine_path, trt_u::Logger& logger)
        : m_super_point(trt_u::load_engine(engine_path, logger), logger)
    {
        //  Input buffers init
        auto in_dims = m_super_point.images_dims();
        m_input_shape = trt_u::Dims3d(in_dims.chan, in_dims.rows, in_dims.cols);
        m_gpu_uint8.create(IMAGE_COUNT, in_dims.rows, in_dims.cols, CV_8UC1);
        m_gpu_float.create(IMAGE_COUNT, in_dims.rows, in_dims.cols, CV_32FC1);
        
        //  Output buffers init
        auto score_map_dims = m_super_point.scores_dims();
        auto descr_map_dims = m_super_point.descrs_dims();
        m_score_map.init(score_map_dims, torch::kFloat32, torch::kCUDA);
        m_descr_map.values = tch_u::make_tensor_4d(descr_map_dims, torch::kFloat32, torch::kCUDA);
    }

    const trt_u::Dims3d& SuperPointDetector::input_shape() const
    {
        return m_input_shape;
    }

    tensor_rt_utils::Dims4d SuperPointDetector::scores_shape() const
    {
        return m_super_point.scores_dims();
    }

    tensor_rt_utils::Dims4d SuperPointDetector::descriptors_shape() const
    {
        return m_super_point.descrs_dims();
    }

    const ScoreMap& SuperPointDetector::score_map() const
    {
        return m_score_map;
    }

    const DescriptorMap& SuperPointDetector::descriptor_map() const
    {
        return m_descr_map;
    }

    void SuperPointDetector::detect(const cv::Mat& first, const cv::Mat& second)
    {
        //  Copy to device, possibly using intermediate buffers (m_gpu_input) to resize on GPU
        if (first.rows != m_input_shape.rows || first.cols != m_input_shape.cols)
        {
            if (first.rows != m_gpu_input[0].rows || first.cols != m_gpu_input[0].cols)
                m_gpu_input[0].create(first.rows, first.cols, first.type());
            m_gpu_input[0].upload(first);
            cv::cuda::resize(m_gpu_input[0], m_gpu_uint8.header[0], 
                cv::Size(m_input_shape.cols, m_input_shape.rows));
        }
        else
            m_gpu_uint8.header[0].upload(first);    
        if (second.rows != m_input_shape.rows || second.cols != m_input_shape.cols)
        {
            if (second.rows != m_gpu_input[1].rows || second.cols != m_gpu_input[1].cols)
                m_gpu_input[1].create(second.rows, second.cols, second.type());
            m_gpu_input[1].upload(second);
            cv::cuda::resize(m_gpu_input[1], m_gpu_uint8.header[1], 
                cv::Size(m_input_shape.cols, m_input_shape.rows));
        }
        else
            m_gpu_uint8.header[1].upload(second);   

        //  Convert to float in range [0, 255]
        for (int i = 0; i < IMAGE_COUNT; ++i)
            m_gpu_uint8.header[i].convertTo(m_gpu_float.header[i], m_gpu_float.header[i].type(), 1.f / 255);

        //  Forward
        void* buffers[] = { m_gpu_float.content.cudaPtr(), 
            m_score_map.values.data_ptr(), m_descr_map.values.data_ptr() };
        m_super_point.execute(buffers);

        m_score_map.image_size[0].width = first.cols;
        m_score_map.image_size[0].height = first.rows;
        m_score_map.image_size[1].width = second.cols;
        m_score_map.image_size[1].height = second.rows;
    }


    //  ------------------------------------------------------------- Performance measurement utilities

    SuperPointDetector::PerformanceStats::PerformanceStats()
        : preprocessing_duration(0)
        , forward_duration(0)
        , call_count(0)
    {}

    void SuperPointDetector::performance_test_detect(const cv::Mat& first, const cv::Mat& second, 
            PerformanceStats& perf_stats)
    {
        namespace chr = std::chrono;
        using Clock = chr::steady_clock;

        static auto diff_time = [](Clock::time_point beg, Clock::time_point end) -> chr::microseconds
        {
            return chr::duration_cast<chr::microseconds>(end - beg);
        };


        auto on_preprocess = chr::steady_clock::now();

                //  Copy to device, possibly using intermediate buffers (m_gpu_input) to resize on GPU
        if (first.rows != m_input_shape.rows || first.cols != m_input_shape.cols)
        {
            if (first.rows != m_gpu_input[0].rows || first.cols != m_gpu_input[0].cols)
                m_gpu_input[0].create(first.rows, first.cols, first.type());
            m_gpu_input[0].upload(first);
            cv::cuda::resize(m_gpu_input[0], m_gpu_uint8.header[0], 
                cv::Size(m_input_shape.cols, m_input_shape.rows));
        }
        else
            m_gpu_uint8.header[0].upload(first);    
        if (second.rows != m_input_shape.rows || second.cols != m_input_shape.cols)
        {
            if (second.rows != m_gpu_input[1] .rows || second.cols != m_gpu_input[1].cols)
                m_gpu_input[1].create(second.rows, second.cols, second.type());
            m_gpu_input[1].upload(second);
            cv::cuda::resize(m_gpu_input[1], m_gpu_uint8.header[1], 
                cv::Size(m_input_shape.cols, m_input_shape.rows));
        }
        else
            m_gpu_uint8.header[1].upload(second);    

        //  Convert to float in range [0, 255]
        for (int i = 0; i < IMAGE_COUNT; ++i)
            m_gpu_uint8.header[i].convertTo(m_gpu_float.header[i], m_gpu_float.header[i].type(), 1.f / 255);

        auto on_forward = chr::steady_clock::now();

        //  Forward
        void* buffers[] = { m_gpu_float.content.cudaPtr(), 
            m_score_map.values.data_ptr(), m_descr_map.values.data_ptr() };
        m_super_point.execute(buffers);
        cudaDeviceSynchronize();

        auto on_exit = chr::steady_clock::now();
        if (perf_stats.call_count > 0)
        {
            perf_stats.preprocessing_duration += diff_time(on_preprocess, on_forward);
            perf_stats.forward_duration += diff_time(on_forward, on_exit);
        }
        ++perf_stats.call_count;
    }
}