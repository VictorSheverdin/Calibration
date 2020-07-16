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

    virtual cv::Mat match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::set< PointTrackResult > *trackedPoints ) = 0;
    virtual cv::Mat track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::set< PointTrackResult > *trackedPoints ) = 0;

    size_t count() const;
    void setCount( const size_t value );

    double minDistance() const;
    void setMinDistance( const double value );

    double extractPrecision() const;
    void setExtractPrecision( const double value );

    size_t winSize() const;
    void setWinSize( const size_t value );

    size_t levels() const;
    void setLevels( const size_t value );

protected:
    FlowTracker() = default;

    void prepareStereoPoints(const FlowFramePtr &frame, std::vector< cv::Point2f > *points, std::set< PointTrackResult > *trackedPoints );
    void prepareConsecutivePoints( const FlowFramePtr &frame, std::vector< cv::Point2f > *points, std::set< PointTrackResult > *trackedPoints );

    std::unique_ptr< FlowProcessor > m_pointsProcessor;

};

class GPUFlowTracker : public FlowTracker
{
public:
    GPUFlowTracker();

    virtual void buildPyramid( FlowFrame *frame ) override;
    virtual void extractPoints( FlowFrame *frame ) override;

    virtual cv::Mat match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::set< PointTrackResult > *trackedPoints ) override;
    virtual cv::Mat track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::set< PointTrackResult > *trackedPoints ) override;

protected:
    GPUFlowProcessor *processor() const;

private:
    void initialize();

};

class CPUFlowTracker : public FlowTracker
{
public:
    CPUFlowTracker();

    virtual void buildPyramid( FlowFrame *frame ) override;
    virtual void extractPoints( FlowFrame *frame ) override;

    virtual cv::Mat match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::set< PointTrackResult > *trackedPoints ) override;
    virtual cv::Mat track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::set< PointTrackResult > *trackedPoints ) override;

protected:
    CPUFlowProcessor *processor() const;

private:
    void initialize();

};

class FeatureTracker
{
public:
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;
    using FeaturePointPtr = std::shared_ptr< FeaturePoint >;

    virtual void extractKeypoints( FeatureFrame *frame ) = 0;
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
    virtual void extractKeypoints( FeatureFrame *frame ) override;

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
