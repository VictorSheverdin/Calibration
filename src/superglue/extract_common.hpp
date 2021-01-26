#ifndef _EXTRACT_COMMON_HPP_
#define _EXTRACT_COMMON_HPP_

#include <vector>
#include <array>

#include "opencv2/core.hpp"
#include "qttorch.h"
#include "libtorch_utils.hpp"


namespace marker
{
    struct KeypointSet
    {
        at::Tensor keypoints;
        at::Tensor scores;
        at::Tensor descriptors;

        void get_keypoints(std::vector<cv::KeyPoint>& dst) const;
        void set_keypoints(const std::vector<cv::KeyPoint>& src);
        void get_descriptors(cv::Mat& dst) const;
        void set_descriptors(const cv::Mat& src);
    };

    using KeypointSetArray = std::array<KeypointSet, 2>;

    struct MatchTable
    {
        at::Tensor scores;

        void get_scores(cv::Mat& dst) const;
    };
}

#endif // _EXTRACT_COMMON_HPP_
