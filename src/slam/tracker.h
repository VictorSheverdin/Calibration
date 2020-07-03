#pragma once

#include "src/common/featureprocessor.h"

namespace slam {

class FeatureFrame;
class FeaturePoint;
class FlowFrame;

class FlowTracker
{
public:
    using FlowFramePtr = std::shared_ptr< FlowFrame >;

    virtual void buildPyramid( FlowFrame *frame ) = 0;
    virtual void extractPoints( FlowFrame *frame ) = 0;

    virtual cv::Mat match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap ) = 0;
    virtual cv::Mat track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap ) = 0;

protected:
    FlowTracker() = default;

    void prepareStereoPoints( const FlowFramePtr &frame, std::vector< cv::Point2f > *points, std::map< size_t, cv::Point2f > *trackedMap );
    void prepareConsecutivePoints( const FlowFramePtr &frame, std::vector< cv::Point2f > *points, std::map< size_t, cv::Point2f > *trackedMap );

};

class GPUFlowTracker : public FlowTracker
{
public:
    GPUFlowTracker() = default;

    virtual void buildPyramid( FlowFrame *frame ) override;
    virtual void extractPoints( FlowFrame *frame ) override;

    virtual cv::Mat match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map<size_t, cv::Point2f> *trackedMap ) override;
    virtual cv::Mat track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap ) override;

    size_t count() const;
    void setCount( const size_t value );

protected:
    GPUFlowProcessor m_pointsProcessor;

};

class CPUFlowTracker : public FlowTracker
{
public:
    CPUFlowTracker() = default;

    virtual void buildPyramid( FlowFrame *frame ) override;
    virtual void extractPoints( FlowFrame *frame ) override;

    virtual cv::Mat match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap ) override;
    virtual cv::Mat track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap ) override;

    size_t count() const;
    void setCount( const size_t value );

protected:
    CPUFlowProcessor m_pointsProcessor;

};

class FeatureTracker
{
public:
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;
    using FeaturePointPtr = std::shared_ptr< FeaturePoint >;

    virtual void prepareFrame( FeatureFrame *frame ) = 0;
    virtual cv::Mat match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches ) = 0;

protected:
    FeatureTracker() = default;

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
