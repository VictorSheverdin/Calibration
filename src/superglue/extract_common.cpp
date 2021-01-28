#include "extract_common.hpp"


namespace marker
{
    namespace trt_u = tensor_rt_utils;
    namespace tch_u = libtorch_utils;

    void ScoreMap::init(tensor_rt_utils::Dims4d dims, c10::ScalarType dtype, c10::Device device)
    {
        values = tch_u::make_tensor_4d(dims, dtype, device);
        image_size.resize(dims.batch);
    }

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

    DescriptorSetArray sample_descriptors(const DescriptorMap& descriptor_map, 
        const KeypointSetArray& keypoints)
    {
        namespace tf = torch::nn::functional;

        static auto bilinear_sample = tf::GridSampleFuncOptions()
            .mode(torch::kBilinear)
            .align_corners(true);
        static auto normalize_l2 = tf::NormalizeFuncOptions()
            .dim(1)
            .p(2);
        DescriptorSetArray ret;
        for(int i = 0; i < keypoints.size(); ++i)
        {
            auto dmap = descriptor_map.values.select(0, i);
            auto grid = keypoints[i].keypoints.clone();

            // Normalize x to [-1, 1]
            auto scale_x = keypoints[i].image_size.width / dmap.size(2);
            grid.slice(1, 0, 1)
                .sub_(scale_x / 2 - .5f)
                .true_divide_(keypoints[i].image_size.width - scale_x / 2 - .5f)
                .mul_(2)
                .sub_(1);

            //  Normalize y to [-1, 1]
            auto scale_y = keypoints[i].image_size.height / dmap.size(1);
            grid.slice(1, 1, 2)
                .sub_(scale_y / 2 - .5f)
                .true_divide_(keypoints[i].image_size.height - scale_y / 2 - .5f)
                .mul_(2)
                .sub_(1);

            //  Perform grid sample and return normalized version
            auto src_sampled = tf::grid_sample(dmap.view({ 1, dmap.size(0), dmap.size(1), dmap.size(2) }), 
                grid.view({1, 1, -1, 2}), bilinear_sample);
            ret[i].values = tf::normalize(src_sampled, normalize_l2).select(0, 0).select(1, 0);
        }
        return ret;
    }
}