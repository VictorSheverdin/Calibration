#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "framepoint.h"
#include "mappoint.h"

#include "src/common/calibrationdatabase.h"

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

class MonoFrame : public FrameBase, public ProjectionMatrix
{
public:
    using PointPtr = std::shared_ptr< MonoPoint >;

    virtual std::vector< PointPtr > framePoints() const = 0;

    std::vector< cv::Point2f > points() const;

    std::vector< StereoPoint > stereoPoints() const;
    std::vector< AdjacentPoint > adjacentPoints() const;

    bool drawPoints( CvImage *target ) const;

    cv::Point3d point() const;

protected:
    MonoFrame();

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

    void cleanPoints();
    void cleanMapPoints();

    void triangulatePoints();

    const std::vector< cv::KeyPoint > &keyPoints() const;

    CvImage drawKeyPoints() const;
    CvImage drawTracks() const;

    static void setMaxFeatures( const int value );
    static int maxFeatures();

    std::vector< ProcessedPointPtr > posePoints() const;
    int posePointsCount() const;

    std::vector< ProcessedPointPtr > trackedPoints() const;

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

    static const double m_cameraDistanceMultiplier;
    static const double m_minPointsDistance;

    static const int m_minConnectedPoints = 4;

    std::vector< ProcessedPointPtr > pointsToTrack() const;

    ProcessedPointPtr createFramePoint( const size_t keyPointIndex, const cv::Scalar &color );

private:
    void initialize();

};

class Frame : public MonoFrame, public std::enable_shared_from_this< Frame >
{
public:
    using FramePtr = std::shared_ptr< Frame >;
    using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;

    using FramePointPtr = std::shared_ptr< FramePoint >;

    virtual std::vector< PointPtr > framePoints() const override;

    static FramePtr create();

    void replace( const ProcessedFramePtr &frame );
    void replaceAndClean( const ProcessedFramePtr &frame );

protected:
    Frame();

    std::list< FramePointPtr > m_points;

    FramePointPtr createFramePoint( const cv::Point2f &point, const cv::Scalar &color );

private:
};

class DoubleFrame : public FrameBase
{
public:
    using MonoFramePtr = std::shared_ptr< MonoFrame >;
    using MonoPointPtr = std::shared_ptr< MonoPoint >;

    void setFrames( const MonoFramePtr &frame1, const MonoFramePtr &frame2 );

    MonoFramePtr frame1() const;
    MonoFramePtr frame2() const;

    void setProjectionMatrix( const ProjectionMatrix &matrix1, const ProjectionMatrix &matrix2 );

    cv::Point3f center() const;

protected:
    DoubleFrame();

    MonoFramePtr m_frame1;
    MonoFramePtr m_frame2;

};

class ProcessedDoubleFrameBase
{
public:
    using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;
    using ProcessedPointPtr = std::shared_ptr< ProcessedPoint >;

    // size_t usedKeypointsCount() const;

protected:
    ProcessedDoubleFrameBase();

    static FeatureMatcher m_featuresMatcher;
    static OpticalMatcher m_opticalMatcher;

    // size_t m_usedKeypointsCount;

private:
    void initialize();

};

class StereoFrameBase : public DoubleFrame
{
public:
    void setFrames( const MonoFramePtr &leftFrame, const MonoFramePtr &rightFrame );

    MonoFramePtr leftFrame() const;
    MonoFramePtr rightFrame() const;

protected:
    StereoFrameBase();

private:
};

class ProcessedStereoFrame : public StereoFrameBase, public ProcessedDoubleFrameBase
{
public:

    using MapPtr = std::shared_ptr< Map >;
    using FramePtr = std::shared_ptr< ProcessedStereoFrame >;

    using MapPointPtr = std::shared_ptr< MapPoint >;

    static FramePtr create( const MapPtr &parentMap );

    void load( const CvImage &image1, const CvImage &image2 );

    void setFrames( const ProcessedFramePtr &leftFrame, const ProcessedFramePtr &rightFrame );

    ProcessedFramePtr leftFrame() const;
    ProcessedFramePtr rightFrame() const;

    void setProjectionMatrix( const ProjectionMatrix &leftMatrix, const ProjectionMatrix &rightMatrix );

    cv::Mat matchOptical( const size_t count );
    cv::Mat matchFeatures( const size_t count );

    void clearImages();

    CvImage drawKeyPoints() const;
    CvImage drawStereoPoints() const;
    CvImage drawTracks() const;

    std::vector< StereoPoint > stereoPoints() const;
    int stereoPointsCount() const;

    MapPtr parentMap() const;

    bool triangulatePoints();

    void cleanMapPoints();

    void extractKeyPoints();
    void extractDescriptors();

    size_t leftKeyPointsCount() const;

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    ProcessedStereoFrame( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;

    static const float m_maxYParallax;

};

class AdjacentFrame : public DoubleFrame, public ProcessedDoubleFrameBase
{
public:
    using FramePtr = std::shared_ptr< AdjacentFrame >;

    static FramePtr create();

    cv::Mat trackOptical();
    cv::Mat trackFeatures(const size_t count );

    double recoverPose();

    void setFrames( const ProcessedFramePtr &prevFrame, const ProcessedFramePtr &nextFrame );

    ProcessedFramePtr previousFrame() const;
    ProcessedFramePtr nextFrame() const;

    std::vector< AdjacentPoint > adjacentPoints() const;
    int adjacentPointsCount() const;

    std::vector< AdjacentFrame::ProcessedPointPtr > posePoints() const;
    int posePointsCount() const;

    std::vector< AdjacentFrame::ProcessedPointPtr > trackedPoints() const;
    int trackedPointsCount() const;

    void extractDescriptors();

protected:    
    AdjacentFrame();

};

class StereoFrame : public DoubleFrame
{
public:
    using FramePtr = std::shared_ptr< Frame >;
    using StereoFramePtr = std::shared_ptr< StereoFrame >;
    using ProcessedStereoFramePtr = std::shared_ptr< ProcessedStereoFrame >;

    static StereoFramePtr create();

    void setFrames( const FramePtr &leftFrame, const FramePtr &rightFrame );

    FramePtr leftFrame() const;
    FramePtr rightFrame() const;

    void setProjectionMatrix( const ProjectionMatrix &leftMatrix, const ProjectionMatrix &rightMatrix );

    void replace( const ProcessedStereoFramePtr &frame );
    void replaceAndClean( const ProcessedStereoFramePtr &frame );

protected:
    StereoFrame();

private:
    void initialize();

};

}
