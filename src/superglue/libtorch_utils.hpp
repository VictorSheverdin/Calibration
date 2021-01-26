#ifndef _LIBTORCH_UTILS_HPP_
#define _LIBTORCH_UTILS_HPP_

#include <stdexcept>

#include "qttorch.h"
#include "tensor_rt_utils.hpp"
#include "opencv2/core.hpp"
#include "cuda_runtime.h"


namespace libtorch_utils
{
    namespace trt_utils = tensor_rt_utils;
    

    inline at::Tensor make_tensor_4d(trt_utils::Dims4d dims, c10::ScalarType dtype, c10::Device device)
    {
        auto opt = torch::TensorOptions()
            .dtype(dtype)
            .layout(torch::kStrided)
            .device(device)
            .requires_grad(false);
        return torch::empty({ dims.batch, dims.chan, dims.rows, dims.cols }, opt);
    }

    inline at::Tensor make_tensor_3d(trt_utils::Dims3d dims, c10::ScalarType dtype, c10::Device device)
    {
        auto opt = torch::TensorOptions()
            .dtype(dtype)
            .layout(torch::kStrided)
            .device(device)
            .requires_grad(false);
        return torch::empty({ dims.chan, dims.rows, dims.cols }, opt);
    }

    inline at::Tensor make_tensor_2d(trt_utils::Dims2d dims, c10::ScalarType dtype, c10::Device device)
    {
        auto opt = torch::TensorOptions()
            .dtype(dtype)
            .layout(torch::kStrided)
            .device(device)
            .requires_grad(false);
        return torch::empty({ dims.rows, dims.cols }, opt);
    }

    template<trt_utils::BlockLayout layout>
    at::Tensor make_block_tensor(trt_utils::DimsBlock<layout> dims, c10::ScalarType dtype, c10::Device device)
    {
        auto opt = torch::TensorOptions()
            .dtype(dtype)
            .layout(torch::kStrided)
            .device(device)
            .requires_grad(false);
        if constexpr (layout == trt_utils::BlockLayout::COUNT_FIRST)
            return torch::empty({ dims.count, dims.depth }, opt);
        else if constexpr (layout == trt_utils::BlockLayout::DEPTH_FIRST)
            return torch::empty({ dims.depth, dims.count }, opt);
        else
            throw std::logic_error("Unknown BlockLayout template parameter! Must be COUNT_FIRST or DEPTH_FIRST");
    }

    inline void host_to_device(const cv::Mat& src, at::Tensor& dst)
    {
        auto status = cudaMemcpy(dst.data_ptr(), src.data, src.total() * src.elemSize(), 
            cudaMemcpyKind::cudaMemcpyHostToDevice);
        if (status != cudaSuccess)
            throw std::runtime_error(cudaGetErrorString(status));
    }

    inline void device_to_host(const at::Tensor& src, cv::Mat& dst)
    {
        auto status = cudaMemcpy(src.data_ptr(), dst.data, dst.total() * dst.elemSize(), 
            cudaMemcpyKind::cudaMemcpyDeviceToHost);
        if (status != cudaSuccess)
            throw std::runtime_error(cudaGetErrorString(status));
    }
}

#endif // _LIBTORCH_UTILS_HPP_
