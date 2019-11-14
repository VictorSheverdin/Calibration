#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "framepoint.h"
#include "worldpoint.h"
#include "src/common/featureprocessor.h"

namespace slam {

class FrameBase
{
protected:
    FrameBase();
    virtual ~FrameBase() = default;

    cv::Mat m_cameraMatrix;

    cv::Mat m_rotation;
    cv::Mat m_translation;

};

class MonoFrame : public FrameBase
{
protected:
    MonoFrame();

};

class FeatureFrame : public MonoFrame, public std::enable_shared_from_this< FeatureFrame >
{
    friend class StereoFrame;
    friend class ConsecutiveFrame;
    friend class FeaturePoint;

public:
    using FramePtr = std::shared_ptr< FeatureFrame >;
    using PointPtr = std::shared_ptr< FeaturePoint >;

    static FramePtr create();
    static FramePtr create( const CvImage &image );

    void load( const CvImage &image );

    size_t pointsCount() const;

    bool drawKeyPoints( CvImage *target ) const;
    bool drawPoints( CvImage *target ) const;

protected:
    FeatureFrame();
    FeatureFrame( const CvImage &image );

    std::vector< cv::KeyPoint > m_keyPoints;
    cv::Mat m_descriptors;

    std::vector< PointPtr > m_framePoints;

    static FeatureProcessor m_processor;

    PointPtr createFramePoint( const size_t keyPointIndex );

};

class Frame : public MonoFrame
{
};

class DoubleFrameBase : public FrameBase
{
public:
    using MonoFramePtr = std::shared_ptr< MonoFrame >;
    using MonoPointPtr = std::shared_ptr< MonoPoint >;

    void setFrames( const MonoFramePtr frame1, const MonoFramePtr frame2 );

    MonoFramePtr frame1() const;
    MonoFramePtr frame2() const;

protected:
    DoubleFrameBase();

    MonoFramePtr m_frame1;
    MonoFramePtr m_frame2;

    static FeatureMatcher m_matcher;

};

class StereoFrame : public DoubleFrameBase
{
    friend class StereoPoint;

public:
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;
    using PointPtr = std::shared_ptr< StereoPoint >;

    using FramePtr = std::shared_ptr< StereoFrame >;

    static FramePtr create();

    void load( const CvImage &leftImage, const CvImage &rightImage );
    void matchFrames( const FeatureFramePtr leftFrame, const FeatureFramePtr rightFrame );

    MonoFramePtr leftFrame() const;
    MonoFramePtr rightFrame() const;

    CvImage drawKeyPoints( const CvImage &leftImage, const CvImage &rightImage ) const;
    CvImage drawStereoPoints( const CvImage &leftImage, const CvImage &rightImage ) const;

protected:
    StereoFrame();

    std::vector< PointPtr > m_framePoints;

    PointPtr createFramePoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint );

private:
};

class ConsecutiveFrame : public DoubleFrameBase
{
    friend class ConsecutivePoint;

public:
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;
    using PointPtr = std::shared_ptr< ConsecutivePoint >;

    using FramePtr = std::shared_ptr< ConsecutiveFrame >;

    static FramePtr create();

    void load( const CvImage &image1, const CvImage &image2 );
    void matchFrames( const FeatureFramePtr frame1, const FeatureFramePtr frame2 );

    CvImage drawTrack( const CvImage &image ) const;

protected:    
    ConsecutiveFrame();

    std::vector< PointPtr > m_framePoints;

    PointPtr createFramePoint( const MonoPointPtr point1, const MonoPointPtr point2 );

};

}
