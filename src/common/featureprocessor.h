#pragma once

#include "src/common/image.h"

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/cudafeatures2d.hpp>

#include <opencv2/optflow.hpp>
#include <opencv2/cudaoptflow.hpp>

class FlowProcessorBase
{
public:
    void extractPoints( const CvImage &image, std::vector< cv::Point2f > *points );

protected:
    FlowProcessorBase() = default;

};

class GPUFlowProcessor : public FlowProcessorBase
{
public:
    GPUFlowProcessor() = default;

};

class CPUFlowProcessor : public FlowProcessorBase
{
public:
    CPUFlowProcessor() = default;

};

class FeatureProcessorBase
{
public:
    virtual ~FeatureProcessorBase() = default;

protected:
    FeatureProcessorBase() = default;

    cv::Ptr< cv::Feature2D > m_processor;

};

class KeyPointProcessor : public FeatureProcessorBase
{
public:
    void extractKeypoints( const CvImage &image, std::vector< cv::KeyPoint > *keypoints );

protected:
    KeyPointProcessor() = default;

};

class DescriptorProcessor : public FeatureProcessorBase
{
public:
    void extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors );

protected:
    DescriptorProcessor() = default;

};

class FullProcessor : public FeatureProcessorBase
{
public:
    void extractKeypoints( const CvImage &image, std::vector< cv::KeyPoint > *keypoints );
    void extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors );

    void extractAndCompute( const CvImage &image, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors );

protected:
    FullProcessor() = default;

};

class GFTTProcessor : public KeyPointProcessor
{
public:
    GFTTProcessor();

    cv::Ptr< cv::GFTTDetector > processor() const;

    void setMaxFeatures( const int value );
    int maxFeatures() const;

private:
    void initialize();

};

class FastProcessor : public KeyPointProcessor
{
public:
    FastProcessor();

    cv::Ptr< cv::FastFeatureDetector > processor() const;

    void setThreshold( const int value );
    int threshold() const;

private:
    void initialize();

};

class DaisyProcessor : public DescriptorProcessor
{
public:
    DaisyProcessor();

private:
    void initialize();

};

class FreakProcessor : public DescriptorProcessor
{
public:
    FreakProcessor();

private:
    void initialize();

};

class SiftProcessor : public FullProcessor
{
public:
    SiftProcessor();

private:
    void initialize();

};

class SurfProcessor : public FullProcessor
{
public:
    SurfProcessor();

private:
    void initialize();

};

class OrbProcessor : public FullProcessor
{
public:
    OrbProcessor();

private:
    void initialize();

};

class KazeProcessor : public FullProcessor
{
public:
    KazeProcessor();

private:
    void initialize();

};

class AKazeProcessor : public FullProcessor
{
public:
    AKazeProcessor();

private:
    void initialize();

};

class FeatureMatcherBase
{
public:
    FeatureMatcherBase();
    virtual ~FeatureMatcherBase() = default;

protected:
private:
};

class DescriptorMatcherBase : public FeatureMatcherBase
{
public:
    DescriptorMatcherBase();

    cv::Mat match( const std::vector<cv::KeyPoint> &queryKeypoints, const cv::Mat &queryDescriptors,
                const std::vector<cv::KeyPoint> &trainKeypoints, const cv::Mat &trainDescriptors,
                std::vector< cv::DMatch > *matches );

protected:
    static double m_threshold;

    cv::Ptr< cv::DescriptorMatcher > m_matcher;

private:
};

class BFMatcher : public DescriptorMatcherBase
{
public:
    BFMatcher();

private:
    void initialize();

};

class FlannMatcher : public DescriptorMatcherBase
{
public:
    FlannMatcher();

private:
    void initialize();

};

class OpticalMatcherBase : public FeatureMatcherBase
{
public:
protected:
    OpticalMatcherBase() = default;

    static const double m_maxDistance;
    static const double m_errorRatio;
};

class GPUOpticalMatcher : public OpticalMatcherBase
{
public:
    GPUOpticalMatcher();

    cv::Mat match( const CvImage &sourceImage, const std::vector< cv::KeyPoint > &sourceKeypoints,
                const CvImage &targetImage, const std::vector< cv::KeyPoint > &targetKeypoints, const cv::Mat &targetSearchMatrix,
                std::vector< cv::DMatch > *matches );

protected:
    cv::Ptr< cv::cuda::SparsePyrLKOpticalFlow > m_opticalProcessor;

    cv::cuda::GpuMat m_gpuSourceImage;
    cv::cuda::GpuMat m_gpuTargetImage;
    cv::cuda::GpuMat m_gpuSourcePoints;
    cv::cuda::GpuMat m_gpuOpticalPoints;
    cv::cuda::GpuMat m_gpuCheckPoints;
    cv::cuda::GpuMat m_gpuStatuses;
    cv::cuda::GpuMat m_gpuCheckStatuses;
    cv::cuda::GpuMat m_gpuErr;
    cv::cuda::GpuMat m_gpuCheckErr;

private:
    void initialize();

};

class CPUOpticalMatcher : public OpticalMatcherBase
{
public:
    CPUOpticalMatcher() = default;

    cv::Mat match( const std::vector< cv::Mat > &sourceImagePyramid, const std::vector< cv::KeyPoint > &sourceKeypoints,
                const std::vector< cv::Mat > &targetImagePyramid, const std::vector< cv::KeyPoint > &targetKeypoints, const cv::Mat &targetSearchMatrix,
                std::vector< cv::DMatch > *matches );

    void buildImagePyramid( const CvImage &image, std::vector< cv::Mat > *imagePyramid );

};
