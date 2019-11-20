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
};

class MonoFrame : public FrameBase
{
public:
    using PointPtr = std::shared_ptr< MonoPoint >;

    std::vector< PointPtr > &framePoints();
    const std::vector< PointPtr > &framePoints() const;

    std::vector< cv::Point2f > points() const;
    std::vector< StereoPoint > stereoPoints() const;

    std::vector< AdjacentPoint > adjacentPoints() const;

    PointPtr framePoint( const size_t index ) const;

    size_t pointsCount() const;
    size_t tracksCount() const;

    bool drawPoints( CvImage *target ) const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void setRotation( const cv::Mat &value );
    const cv::Mat rotation() const;

    void setTranslation( const cv::Mat &value );
    const cv::Mat translation() const;

    void setProjectionMatrix( const cv::Mat &value );
    const cv::Mat &projectionMatrix() const;

protected:
    MonoFrame();

    std::vector< PointPtr > m_points;

    cv::Mat m_cameraMatrix;

    cv::Mat m_rtMatrix;

    mutable cv::Mat m_projectionMatrix;

private:
    void initialize();

};

class FeatureFrame : public MonoFrame, public std::enable_shared_from_this< FeatureFrame >
{
    friend class StereoFrame;
    friend class AdjacentFrame;
    friend class FeaturePoint;

public:
    using FramePtr = std::shared_ptr< FeatureFrame >;
    using PointPtr = std::shared_ptr< FeaturePoint >;

    static FramePtr create();

    void load( const CvImage &image );

    bool drawKeyPoints( CvImage *target ) const;

protected:
    FeatureFrame();

    std::vector< cv::KeyPoint > m_keyPoints;
    cv::Mat m_descriptors;

    static FeatureProcessor m_processor;

    PointPtr createFramePoint( const size_t keyPointIndex, const cv::Scalar &color );

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
public:
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;
    using FramePtr = std::shared_ptr< StereoFrame >;

    static FramePtr create();

    void load( const CvImage &leftImage, const CvImage &rightImage );
    void matchFrames();

    MonoFramePtr leftFrame() const;
    MonoFramePtr rightFrame() const;

    std::vector< StereoPoint > stereoPoints() const;

    CvImage drawKeyPoints( const CvImage &leftImage, const CvImage &rightImage ) const;
    CvImage drawStereoPoints( const CvImage &leftImage, const CvImage &rightImage ) const;

protected:
    StereoFrame();

};

class World;

class WorldStereoFrame : public StereoFrame
{
public:
    using WorldPtr = std::shared_ptr< World >;
    using FramePtr = std::shared_ptr< WorldStereoFrame >;

    static FramePtr create( const WorldPtr &parentWorld );

    const WorldPtr parentWorld() const;

    void triangulatePoints();

protected:
    using WorldPtrImpl = std::weak_ptr< World >;

    WorldStereoFrame( const WorldPtr &parentWorld );

    WorldPtrImpl m_parentWorld;

};

class AdjacentFrame : public DoubleFrameBase
{
public:
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;
    using FramePtr = std::shared_ptr< AdjacentFrame >;

    static FramePtr create();

    void load( const CvImage &image1, const CvImage &image2 );
    void matchFrames( const FeatureFramePtr frame1, const FeatureFramePtr frame2 );

    MonoFramePtr previousFrame() const;
    MonoFramePtr nextFrame() const;

    std::vector< AdjacentPoint > adjacentPoints() const;

    CvImage drawTrack( const CvImage &image ) const;

protected:    
    AdjacentFrame();

};

class WorldAdjacentFrame : public AdjacentFrame
{
public:
    using WorldPtr = std::shared_ptr< World >;
    using FramePtr = std::shared_ptr< WorldAdjacentFrame >;

    static FramePtr create( const WorldPtr &parentWorld );

    const WorldPtr parentWorld() const;

    void triangulatePoints();

protected:
    using WorldPtrImpl = std::weak_ptr< World >;

    WorldAdjacentFrame( const WorldPtr &parentWorld );

    WorldPtrImpl m_parentWorld;

};

}
