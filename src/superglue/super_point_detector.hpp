#ifndef _SUPER_POINT_DETECTOR_HPP_
#define _SUPER_POINT_DETECTOR_HPP_

#include <string>
#include <vector>
#include <array>
#include <chrono>

#include "opencv2/core.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"

#include "qttorch.h"

#include "extract_common.hpp"
#include "super_point.hpp"
#include "tensor_rt_utils.hpp"
#include "libtorch_utils.hpp"


namespace marker
{
    class SuperPointDetector
    {
    public:
        static constexpr int IMAGE_COUNT = 1;

    public:
        SuperPointDetector(const std::string& engine_path, tensor_rt_utils::Logger& logger);
        const tensor_rt_utils::Dims3d& input_shape() const;
        const tensor_rt_utils::Dims4d& scores_shape() const;
        const tensor_rt_utils::Dims4d& descriptors_shape() const;

        void detect(const cv::Mat& image);

        const ScoreMap& score_map() const;
        const DescriptorMap& descriptor_map() const;

    public:
        struct PerformanceStats
        {
            std::chrono::microseconds preprocessing_duration;
            std::chrono::microseconds forward_duration;
            std::size_t call_count;

            PerformanceStats();
        };
        void performance_test_detect(const cv::Mat& image, PerformanceStats& perf_stats);

    private:
        SuperPoint m_super_point;
        
        cv::cuda::GpuMat m_gpu_input;
        cv::cuda::GpuMat m_gpu_uint8;
        cv::cuda::GpuMat m_gpu_float;

        tensor_rt_utils::Dims3d m_input_shape;
        tensor_rt_utils::Dims4d m_scores_shape;
        tensor_rt_utils::Dims4d m_descr_shape;

        ScoreMap m_score_map;
        DescriptorMap m_descr_map;

    private:
        void upload_image(const cv::Mat& image);
        void convert_normalize();
        void forward();
        void read_image_size(const cv::Mat& image);
    };
}

#endif // _SUPER_POINT_DETECTOR_HPP_
