#ifndef _GPU_MAT_BATCH_HPP_
#define _GPU_MAT_BATCH_HPP_

#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudaimgproc.hpp"


namespace marker
{
    struct GpuMatBatch
    {
        std::vector<cv::cuda::GpuMat> header;
        cv::cuda::GpuMat content;

        GpuMatBatch() = default;

        void create(std::uint32_t size, std::uint32_t rows, std::uint32_t cols, int type)
        {
            content = cv::cuda::createContinuous(size * rows, cols, type);
            for(std::size_t i = 0; i < size; ++i)
                header.push_back(content(cv::Rect(0, i * rows, cols, rows)));
        }
    };
}

#endif // _GPU_MAT_BATCH_HPP_