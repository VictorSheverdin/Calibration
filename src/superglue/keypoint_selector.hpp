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
        void select(const ScoreMap& score_map, int count, KeypointSet& dst);
        void select(const ScoreMap& score_map, const cv::Mat& mask, int count, KeypointSet& dst);

    private:
        cv::Size2i m_scores_size;
        cv::cuda::GpuMat m_input_mask;
        cv::cuda::GpuMat m_input_mask_resized;
        at::Tensor m_score_mask;
        at::Tensor m_inner_area;
        at::Tensor m_topk_indices;
        Config m_cfg;

    private:
        void upload_mask(const cv::Mat& mask);
        void apply_input_mask();
        void apply_threshold_mask(const ScoreMap& score_map);
        void select_inner(const ScoreMap& score_map, int count, KeypointSet& dst);
    };
}

#endif  //  _KEYPOINT_SELECTOR_HPP_
