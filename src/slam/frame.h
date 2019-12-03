#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "framepoint.h"
#include "mappoint.h"
#include "src/common/featureprocessor.h"

namespace slam {

class Map;
class MapPoint;

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

    virtual std::vector< PointPtr > framePoints() const = 0;

    std::vector< cv::Point2f > points() const;

    std::vector< StereoPoint > stereoPoints() const;
    std::vector< AdjacentPoint > adjacentPoints() const;

    bool drawPoints( CvImage *target ) const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2f &value );

    void setRotation( const cv::Mat &value );
    const cv::Mat &rotation() const;

    void setTranslation( const cv::Mat &value );
    const cv::Mat &translation() const;

    void setProjectionMatrix( const cv::Mat &value );
    const cv::Mat &projectionMatrix() const;

    cv::Point3d coordinates() const;

protected:
    MonoFrame();

    cv::Mat m_cameraMatrix;

    cv::Mat m_r;
    cv::Mat m_t;

    mutable cv::Mat m_projectionMatrix;

private:
    void initialize();

};

class ProcessedFrame : public MonoFrame, public std::enable_shared_from_this< ProcessedFrame >
{
    friend class ProcessedStereoFrame;
    friend class AdjacentFrame;
    friend class ProcessedPoint;

public:
    using MapPtr = std::shared_ptr< Map >;

    using MapPointPtr = std::shared_ptr< MapPoint >;

    using MonoFramePtr = std::shared_ptr< MonoFrame >;
    using FramePtr = std::shared_ptr< ProcessedFrame >;

    using ProcessedPointPtr = std::shared_ptr< ProcessedPoint >;

    virtual std::vector< PointPtr > framePoints() const override;

    MapPtr parentMap() const;

    ProcessedPointPtr &framePoint( const size_t index );
    const ProcessedPointPtr &framePoint( const size_t index ) const;

    static FramePtr create( const MapPtr &parentMap );

    void load( const CvImage &image );
    void extractKeyPoints();
    void extractDescriptors();

    const CvImage image() const;

    void clearImage();

    void triangulatePoints();

    const std::vector< cv::KeyPoint > &keyPoints() const;

    CvImage drawKeyPoints() const;

    static void setMaxFeatures( const int value );
    static int maxFeatures();

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    ProcessedFrame( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;

    CvImage m_image;

    std::vector< cv::KeyPoint > m_keyPoints;
    std::vector< cv::Scalar > m_colors;
    cv::Mat m_descriptors;

    std::map< size_t, ProcessedPointPtr > m_points;

    static GFTTProcessor m_keypointProcessor;
    static DaisyProcessor m_descriptorProcessor;

    ProcessedPointPtr createFramePoint( const size_t keyPointIndex, const cv::Scalar &color );

    static const double m_minCameraDistanceFactor;
    static const double m_minPointsDistance;

private:
    void initialize();

};

class Frame : public MonoFrame
{
};

class DoubleFrame : public FrameBase
{
public:
    using MonoFramePtr = std::shared_ptr< MonoFrame >;
    using MonoPointPtr = std::shared_ptr< MonoPoint >;

    void setFrames( const MonoFramePtr &frame1, const MonoFramePtr &frame2 );

    MonoFramePtr frame1() const;
    MonoFramePtr frame2() const;

protected:
    DoubleFrame();

    MonoFramePtr m_frame1;
    MonoFramePtr m_frame2;

};

class ProcessedDoubleFrame : public DoubleFrame
{
public:
    using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;

    void setFrames( const ProcessedFramePtr &frame1, const ProcessedFramePtr &frame2 );

    ProcessedFramePtr frame1() const;
    ProcessedFramePtr frame2() const;

    void extractKeyPoints();
    void extractDescriptors();

protected:
    ProcessedDoubleFrame();

    static FeatureMatcher m_matcher;
    static OpticalMatcher m_opticalMatcher;

};

class ProcessedStereoFrame : public ProcessedDoubleFrame
{
public:
    using MapPtr = std::shared_ptr< Map >;
    using FramePtr = std::shared_ptr< ProcessedStereoFrame >;

    static FramePtr create( const MapPtr &parentMap );

    void load( const CvImage &image1, const CvImage &image2 );

    ProcessedFramePtr leftFrame() const;
    ProcessedFramePtr rightFrame() const;

    cv::Mat matchOptical();
    cv::Mat matchFeatures();

    void clearImages();

    CvImage drawKeyPoints() const;
    CvImage drawStereoPoints() const;

    std::vector< StereoPoint > stereoPoints() const;
    int stereoPointsCount() const;

    MapPtr parentMap() const;

    bool triangulatePoints();

    void clearMapPoints();

    static void setMaxFeatures( const int value );
    static int maxFeatures();

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    ProcessedStereoFrame( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;

    static const int m_minLenght = 1;
    static const float m_maxYParallax;

    void clearMapPoints( const std::vector< MonoPointPtr > &points );

};

class AdjacentFrame : public ProcessedDoubleFrame
{
public:
    using FramePtr = std::shared_ptr< AdjacentFrame >;

    static FramePtr create();

    cv::Mat matchOptical();
    cv::Mat matchFeatures();

    bool track();

    ProcessedFramePtr previousFrame() const;
    ProcessedFramePtr nextFrame() const;

    std::vector< AdjacentPoint > adjacentPoints() const;
    int adjacentPointsCount() const;

    std::vector< MonoPointPtr > trackPoints() const;
    int trackPointsCount() const;

    CvImage drawTrack() const;

protected:    
    AdjacentFrame();

    static const int m_minLenght = 1;
    static const int m_minPnpPoints = 50;

};

class StereoFrame : public DoubleFrame
{
public:
    using FramePtr = std::shared_ptr< StereoFrame >;

    static FramePtr create();

protected:
    StereoFrame();

};

}
