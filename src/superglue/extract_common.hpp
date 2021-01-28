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
        std::vector<cv::Size2i> image_size;

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

        void get_keypoints(std::vector<cv::KeyPoint>& dst) const;
        void set_keypoints(const std::vector<cv::KeyPoint>& src);
    };

    using KeypointSetArray = std::array<KeypointSet, 2>;

    struct DescriptorSet
    {
        at::Tensor values;
    };

    using DescriptorSetArray = std::array<DescriptorSet, 2>;


    DescriptorSetArray sample_descriptors(const DescriptorMap& descriptor_map, 
            const KeypointSetArray& keypoints);

    // struct KeypointSet
    // {
    //     at::Tensor keypoints;
    //     at::Tensor scores;
    //     at::Tensor descriptors;

    //     void get_keypoints(std::vector<cv::KeyPoint>& dst) const;
    //     void set_keypoints(const std::vector<cv::KeyPoint>& src);
    //     void get_descriptors(cv::Mat& dst) const;
    //     void set_descriptors(const cv::Mat& src);
    //     void append(const std::vector<cv::KeyPoint>& kpts, const cv::Mat& descr);
    // };

    // using KeypointSetArray = std::array<KeypointSet, 2>;

    struct MatchTable
    {
        at::Tensor scores;

        void get_scores(cv::Mat& dst) const;
    };
}

#endif // _EXTRACT_COMMON_HPP_
