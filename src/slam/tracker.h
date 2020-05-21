#pragma once

#include "src/common/featureprocessor.h"

namespace slam {

class FeatureFrame;

class FlowTrackerBase
{
protected:
    FlowTrackerBase() = default;
};

class FlowTracker
{
public:
    void extractPoints( const CvImage &image, std::vector< cv::Point2f > *points );

    FlowProcessorBase m_pointsProcessor;

};

class FeatureTracker
{
public:
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;

    virtual void prepareFrame( FeatureFrame *frame ) = 0;
    virtual cv::Mat match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches ) = 0;

protected:
    FeatureTracker() = default;

};

class OpticalTrackerBase : public FeatureTracker
{
public:
    void setMaxFeatures( const int value );
    int maxFeatures();

    virtual void prepareFrame( FeatureFrame *frame ) override;

protected:
    GFTTProcessor m_keypointProcessor;

    OpticalTrackerBase() = default;
};

class GPUOpticalTracker : public OpticalTrackerBase
{
public:
    GPUOpticalTracker() = default;

    virtual cv::Mat match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches ) override;

protected:
    GPUOpticalMatcher m_opticalMatcher;

};

class CPUOpticalTracker : public OpticalTrackerBase
{
public:
    CPUOpticalTracker() = default;

    virtual void prepareFrame( FeatureFrame *frame ) override;
    virtual cv::Mat match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches ) override;

protected:
    CPUOpticalMatcher m_opticalMatcher;

};

class DescriptorTracker : public FeatureTracker
{
public:

protected:
    DescriptorTracker() = default;

    bool selectKeypoints( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors );

};

class FullTracker : public DescriptorTracker
{
public:
    virtual void prepareFrame( FeatureFrame *frame ) override;

protected:
    FullTracker() = default;

    std::unique_ptr< FullProcessor > m_descriptorProcessor;
};

class SiftTracker : public FullTracker
{
public:
    SiftTracker();

    virtual cv::Mat match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches ) override;

protected:
    FlannMatcher m_featuresMatcher;

private:
    void initialize();

};

class OrbTracker : public FullTracker
{
public:
    OrbTracker();

    virtual cv::Mat match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches ) override;

protected:
    BFMatcher m_featuresMatcher;

private:
    void initialize();

};


}
