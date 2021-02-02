#ifndef _EXTRACT_COMMON_HPP_
#define _EXTRACT_COMMON_HPP_

#include <vector>
#include <array>

#include "opencv2/core.hpp"
#include "qttorch.h"
#include "libtorch_utils.hpp"
#include "tensor_rt_utils.hpp"


namespace marker
{   
    struct ScoreMap
    {
        at::Tensor values;
        cv::Size2i image_size;

        void init(tensor_rt_utils::Dims4d dims, c10::ScalarType dtype, c10::Device device);
    };

    struct DescriptorMap
    {
        at::Tensor values;
    };

    struct KeypointSet
    {
        at::Tensor keypoints;
        at::Tensor scores;
        cv::Size2i image_size;

        std::int64_t count() const;
        void download(std::vector<cv::KeyPoint>& dst) const;
        void upload(const std::vector<cv::KeyPoint>& src);

        static KeypointSet create();
    };

    struct DescriptorSet
    {
        at::Tensor values;

        void download(cv::Mat& dst);
        void upload(const cv::Mat& src);
        
        static DescriptorSet create();
    };

    DescriptorSet sample_descriptors(const DescriptorMap& descriptor_map, const KeypointSet& keypoints);

    struct MatchTable
    {
        at::Tensor scores;

        void download(cv::Mat& dst) const;
    };
}

#endif // _EXTRACT_COMMON_HPP_
