#ifndef _KEYPOINT_SELECTOR_HPP_
#define _KEYPOINT_SELECTOR_HPP_

#include <vector>
#include <cstdint>
#include <array>

#include "qttorch.h"

#include "opencv2/core.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"
#include "libtorch_utils.hpp"
#include "tensor_rt_utils.hpp"
#include "extract_common.hpp"


namespace marker
{
    class KeypointSelector
    {
    public:
        static constexpr int IMAGE_COUNT = 2;

        struct Config
        {
            int border;
            float score_threshold;
        };

        KeypointSelector(const Config& cfg, const tensor_rt_utils::Dims4d& scores_dims);
        void select_keypoints(const ScoreMap& score_map, int count, KeypointSetArray& dst);
        void select_keypoints(const ScoreMap& score_map, const cv::Mat& first_mask, 
            const cv::Mat& second_mask, int count, KeypointSetArray& dst);
        static KeypointSetArray make_keypoints();

    private:
        cv::Size2i m_scores_size;
        std::array<cv::cuda::GpuMat, IMAGE_COUNT> m_input_mask;
        std::array<cv::cuda::GpuMat, IMAGE_COUNT> m_input_mask_resized;
        at::Tensor m_score_mask;
        at::Tensor m_inner_area;
        at::Tensor m_topk_indices;
        Config m_cfg;
    };
}

#endif  //  _KEYPOINT_SELECTOR_HPP_
