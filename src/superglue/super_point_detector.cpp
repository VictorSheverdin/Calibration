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
        m_gpu_uint8 = cv::cuda::createContinuous(in_dims.rows, in_dims.cols, CV_8UC1);
        m_gpu_float = cv::cuda::createContinuous(in_dims.rows, in_dims.cols, CV_32FC1);
        
        //  Output buffers init
        m_scores_shape = m_super_point.scores_dims();
        m_descr_shape = m_super_point.descrs_dims();
        m_score_map.init(m_scores_shape, torch::kFloat32, torch::kCUDA);
        m_descr_map.values = tch_u::make_tensor_4d(m_descr_shape, torch::kFloat32, torch::kCUDA);
    }

    const trt_u::Dims3d& SuperPointDetector::input_shape() const
    {
        return m_input_shape;
    }

    const tensor_rt_utils::Dims4d& SuperPointDetector::scores_shape() const
    {
        return m_scores_shape;
    }

    const tensor_rt_utils::Dims4d& SuperPointDetector::descriptors_shape() const
    {
        return m_descr_shape;
    }

    const ScoreMap& SuperPointDetector::score_map() const
    {
        return m_score_map;
    }

    const DescriptorMap& SuperPointDetector::descriptor_map() const
    {
        return m_descr_map;
    }

    void SuperPointDetector::detect(const cv::Mat& image)
    {
        upload_image(image);
        convert_normalize();
        forward();
        read_image_size(image);
    }

    void SuperPointDetector::upload_image(const cv::Mat& image)
    {
        if (image.rows != m_input_shape.rows || image.cols != m_input_shape.cols)
        {
            if (image.rows != m_gpu_input.rows || image.cols != m_gpu_input.cols)
                m_gpu_input.create(image.rows, image.cols, image.type());
            m_gpu_input.upload(image);
            cv::cuda::resize(m_gpu_input, m_gpu_uint8, 
                cv::Size(m_input_shape.cols, m_input_shape.rows));
        }
        else
            m_gpu_uint8.upload(image);
    }

    void SuperPointDetector::convert_normalize()
    {
        //  Convert to float in range [0, 255]
        m_gpu_uint8.convertTo(m_gpu_float, m_gpu_float.type(), 1.f / 255);
    }

    void SuperPointDetector::forward()
    {
        void* buffers[] = { m_gpu_float.cudaPtr(), 
            m_score_map.values.data_ptr(), m_descr_map.values.data_ptr() };
        m_super_point.execute(buffers);
    }

    void SuperPointDetector::read_image_size(const cv::Mat& image)
    {
        m_score_map.image_size.width = image.cols;
        m_score_map.image_size.height = image.rows;
    }


    //  ------------------------------------------------------------- Performance measurement utilities

    SuperPointDetector::PerformanceStats::PerformanceStats()
        : preprocessing_duration(0)
        , forward_duration(0)
        , call_count(0)
    {}

    void SuperPointDetector::performance_test_detect(const cv::Mat& image, PerformanceStats& perf_stats)
    {
        namespace chr = std::chrono;
        using Clock = chr::steady_clock;

        static auto diff_time = [](Clock::time_point beg, Clock::time_point end) -> chr::microseconds
        {
            return chr::duration_cast<chr::microseconds>(end - beg);
        };

        auto on_preprocess = Clock::now();

        upload_image(image);
        convert_normalize();

        auto on_forward = Clock::now();

        forward();
        cudaDeviceSynchronize();

        read_image_size(image);

        auto on_exit = Clock::now();
        
        if (perf_stats.call_count > 0)
        {
            perf_stats.preprocessing_duration += diff_time(on_preprocess, on_forward);
            perf_stats.forward_duration += diff_time(on_forward, on_exit);
        }
        ++perf_stats.call_count;
    }
}