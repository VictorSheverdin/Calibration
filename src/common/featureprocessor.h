#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "src/common/image.h"

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

class FeatureMatcherBase
{
public:
    FeatureMatcherBase();
    virtual ~FeatureMatcherBase() = default;

protected:
private:
};

class FeatureMatcher : public FeatureMatcherBase
{
public:
    FeatureMatcher();

    cv::Mat match( std::vector<cv::KeyPoint> &queryKeypoints, const cv::Mat &queryDescriptors,
                std::vector<cv::KeyPoint> &trainKeypoints, const cv::Mat &trainDescriptors,
                std::vector< cv::DMatch > *matches );

protected:
    cv::Ptr< cv::DescriptorMatcher > m_matcher;

    static double m_threshold;

private:
    void initialize();

};

class OpticalMatcher : public FeatureMatcherBase
{
public:
    OpticalMatcher();

    cv::Mat match( const CvImage &sourceImage, std::vector<cv::KeyPoint> &sourceKeypoints,
                const CvImage &targetImage, std::vector<cv::KeyPoint> &targetKeypoints,
                std::vector< cv::DMatch > *matches );

protected:
    cv::Ptr< cv::cuda::SparsePyrLKOpticalFlow > m_opticalProcessor;

    cv::cuda::GpuMat m_gpuSourceImage;
    cv::cuda::GpuMat m_gpuTargetImage;
    cv::cuda::GpuMat m_gpuSourcePoints;
    cv::cuda::GpuMat m_gpuOpticalPoints;
    cv::cuda::GpuMat m_gpuStatuses;
    cv::cuda::GpuMat m_gpuErr;

    static const int m_minPointsCount = 10;
    static const double m_maxDistance;

private:
    void initialize();
};
