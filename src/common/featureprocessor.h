#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "src/common/image.h"

class FeatureProcessorBase
{
public:
    FeatureProcessorBase();
    virtual ~FeatureProcessorBase() = default;

protected:
private:
};

class FeatureProcessor : public FeatureProcessorBase
{
public:
    FeatureProcessor();

    void extract(const CvImage &image, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors );

protected:
    cv::Ptr<cv::Feature2D> m_detector;
    cv::Ptr<cv::Feature2D> m_descriptor;

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

