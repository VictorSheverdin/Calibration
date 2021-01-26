#include "extract_common.hpp"


namespace marker
{
    namespace trt_u = tensor_rt_utils;
    namespace tch_u = libtorch_utils;

    void KeypointSet::get_keypoints(std::vector<cv::KeyPoint>& dst) const
    {
        //  Copy to host and get accessors
        auto cpu_keypoints = keypoints.to(torch::kCPU);
        auto cpu_scores = scores.to(torch::kCPU);
        auto keypoints_accessor = cpu_keypoints.accessor<float, 2>();
        auto scores_accessor = cpu_scores.accessor<float, 1>();
        
        //  Write to dst
        auto count = keypoints.size(0);
        dst.resize(count);
        for(int i = 0; i < count; ++i)
        {
            dst[i].pt.x = keypoints_accessor[i][0];
            dst[i].pt.y = keypoints_accessor[i][1];
            dst[i].response = scores_accessor[i];
        }
    }

    void KeypointSet::get_descriptors(cv::Mat& dst) const
    {   
        //  Copy to host
        auto cpu_descr = descriptors.to(torch::kCPU);

        //  Prepare dst and copy data
        dst.create(descriptors.size(0), descriptors.size(1), CV_32FC1);
        auto copy_src = static_cast<float*>(cpu_descr.data_ptr());
        auto copy_count = dst.rows * dst.cols;
        auto copy_dst = dst.ptr<float>(0);
        std::copy_n(copy_src, copy_count, copy_dst);
    }

    void MatchTable::get_scores(cv::Mat& dst) const
    {
        //  Copy to host
        auto cpu_scores = scores.squeeze(0).to(torch::kCPU);

        //  Prepare dst and copy data
        dst.create(cpu_scores.size(0), cpu_scores.size(1), CV_32FC1);
        auto copy_src = static_cast<float*>(cpu_scores.data_ptr());
        auto copy_count = dst.rows * dst.cols;
        auto copy_dst = dst.ptr<float>(0);
        std::copy_n(copy_src, copy_count, copy_dst);
    }

    void KeypointSet::set_keypoints(const std::vector<cv::KeyPoint>& src)
    {   
        //  Copy keypoint data to contiguous memory
        cv::Mat cpu_keypoints(src.size(), 2, CV_32FC1);
        auto keypoints_ptr = cpu_keypoints.ptr<float>(0);
        cv::Mat cpu_scores(src.size(), 1, CV_32FC1);
        auto scores_ptr = cpu_scores.ptr<float>(0);
        for(int i = 0; i < src.size(); ++i)
        {
            keypoints_ptr[2 * i + 0] = src[i].pt.x;
            keypoints_ptr[2 * i + 1] = src[i].pt.y;
            scores_ptr[i] = src[i].response;
        }
        
        //  Prepare device buffers and copy data
        this->keypoints = tch_u::make_tensor_2d(trt_u::Dims2d(src.size(), 2), torch::kFloat32, torch::kCUDA);
        this->scores = tch_u::make_tensor_2d(trt_u::Dims2d(src.size(), 1), torch::kFloat32, torch::kCUDA);
        tch_u::host_to_device(cpu_keypoints, this->keypoints);
        tch_u::host_to_device(cpu_scores, this->scores);
    }

    void KeypointSet::set_descriptors(const cv::Mat& src)
    {
        //  Prepare device buffers and copy data
        this->descriptors = tch_u::make_tensor_2d(trt_u::Dims2d(src.rows, src.cols), torch::kFloat32, torch::kCUDA);
        tch_u::host_to_device(src, this->descriptors);
    }
}