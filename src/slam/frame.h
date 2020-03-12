#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "framepoint.h"
#include "mappoint.h"

#include "src/common/projectionmatrix.h"
#include "src/common/featureprocessor.h"
#include "src/common/stereoprocessor.h"

#include <g2o/types/slam3d/se3quat.h>

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
    friend class DoubleFrame;

public:
    using FramePtr = std::shared_ptr< MonoFrame >;
    using PointPtr = std::shared_ptr< MonoPoint >;

    virtual std::vector< PointPtr > framePoints() const = 0;

    std::vector< cv::Point2f > points() const;

    std::vector< StereoPoint > stereoPoints() const;
    std::vector< AdjacentPoint > adjacentPoints() const;

    bool drawPoints( CvImage *target ) const;

    cv::Point3d point() const;

    void setSe3Pose( const g2o::SE3Quat &pose );
    g2o::SE3Quat se3Pose() const;

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

    std::vector< ProcessedPointPtr > processedPoints() const;

    MapPtr parentMap() const;

    ProcessedPointPtr &framePoint( const size_t index );
    const ProcessedPointPtr &framePoint( const size_t index ) const;

    static FramePtr create( const MapPtr &parentMap );

    void load( const CvImage &image );
    void extractKeyPoints();
    void extractGradientPoints();
    void extractDescriptors();

    const CvImage image() const;

    void clearImage();

    void cleanPoints();
    void cleanMapPoints();

    int triangulatePoints();

    const std::vector< cv::KeyPoint > &keyPoints() const;
    size_t keyPointsCount() const;

    CvImage drawPoints() const;
    CvImage drawTracks() const;

    static void setMaxFeatures( const int value );
    static int maxFeatures();

    std::vector< ProcessedPointPtr > posePoints() const;
    int posePointsCount() const;

    std::vector< ProcessedPointPtr > trackFramePoints() const;
    int trackFramePointsCount() const;

    std::vector< ProcessedPointPtr > trackedPoints() const;
    int trackedPointsCount() const;

    void createFramePoints( const size_t count );

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    ProcessedFrame( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;

    CvImage m_image;

    std::vector< cv::KeyPoint > m_keyPoints;
    std::vector< cv::Scalar > m_colors;
    cv::Mat m_descriptors;

    std::vector< cv::Point2f > m_gradientPoints;

    std::map< size_t, ProcessedPointPtr > m_points;

    static GFTTProcessor m_keypointProcessor;
    static DaisyProcessor m_descriptorProcessor;
    static GradientProcessor m_gradientProcessor;

    static const double m_cameraDistanceMultiplier;
    static const double m_minPointsDistance;

    static const int m_minConnectedPoints = 7;

    ProcessedPointPtr createFramePoint( const size_t keyPointIndex );

    bool isFramePointExist( const size_t index ) const;

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
    friend class Map;

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

protected:
    ProcessedDoubleFrameBase();

    static FeatureMatcher m_featuresMatcher;
    static OpticalMatcher m_opticalMatcher;

private:
    void initialize();

};

class StereoFrameBase : public DoubleFrame
{
public:
    using MapPtr = std::shared_ptr< Map >;

    using FramePtr = std::shared_ptr< StereoFrameBase >;

    void setFrames( const MonoFramePtr &leftFrame, const MonoFramePtr &rightFrame );

    void setProjectionMatrix( const StereoCameraMatrix &matrix );
    StereoCameraMatrix projectionMatrix() const;

    double bf() const;

    MonoFramePtr leftFrame() const;
    MonoFramePtr rightFrame() const;

    void setLeftSe3Pose( g2o::SE3Quat &pose );

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    StereoFrameBase( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;

private:
};

class ProcessedStereoFrame : public StereoFrameBase, public ProcessedDoubleFrameBase
{
public:
    using FramePtr = std::shared_ptr< ProcessedStereoFrame >;

    using MapPointPtr = std::shared_ptr< MapPoint >;

    static FramePtr create( const MapPtr &parentMap );

    void load( const CvImage &image1, const CvImage &image2 );

    void setFrames( const ProcessedFramePtr &leftFrame, const ProcessedFramePtr &rightFrame );

    ProcessedFramePtr leftFrame() const;
    ProcessedFramePtr rightFrame() const;

    cv::Mat matchOptical( const size_t count );
    cv::Mat matchFeatures();

    void clearImages();

    CvImage drawPoints() const;
    CvImage drawStereoCorrespondences() const;
    CvImage drawTracks() const;

    std::vector< StereoPoint > stereoPoints() const;
    int stereoPointsCount() const;

    MapPtr parentMap() const;

    int triangulatePoints();

    void cleanMapPoints();

    void extractKeyPoints();
    void extractGradientPoints();
    void extractDescriptors();

    size_t leftKeyPointsCount() const;

protected:
    ProcessedStereoFrame( const MapPtr &parentMap );

    static const double m_minXDistasnce;

};

class DenseFrameBase
{
    friend class DenseFrame;

public:
    DenseFrameBase();
    virtual ~DenseFrameBase();

    void setPoints( const std::list< ColorPoint3d > &list );
    const std::list< ColorPoint3d > &points() const;

    static void setDisparityToDepthMatrix( const cv::Mat & mat );

    static int getMinDisparity();
    static void setMinDisparity( const int minDisparity );

    static int getNumDisparities();
    static void setNumDisparities( const int numDisparities );

    static int getBlockSize();
    static void setBlockSize( const int blockSize );

    static int getTextureThreshold();
    static void setTextureThreshold( const int textureThreshold );

    static int getSpeckleWindowSize();
    static void setSpeckleWindowSize( const int speckleWindowSize );

    static int getSpeckleRange();
    static void setSpeckleRange( const int speckleRange );

    static int getDisp12MaxDiff();
    static void setDisp12MaxDiff( const int disp12MaxDiff );

    static int getPreFilterSize();
    static void setPreFilterSize( const int preFilterSize );

    static int getPreFilterCap();
    static void setPreFilterCap( const int preFilterCap );

    static int getUniquenessRatio();
    static void setUniquenessRatio( const int uniquenessRatio );

protected:
    using OptimizationGrid = std::map< int, std::map< int, std::map< int, std::list< ColorPoint3d > > > >;

    std::list< ColorPoint3d > m_points;

    OptimizationGrid m_optimizationGrid;

    static BMStereoProcessor m_stereoProcessor;

    void createOptimizationGrid();
    void setOptimizationGrid( const OptimizationGrid &grid );

private:
    void initialize();

};

class ProcessedDenseFrame : public ProcessedStereoFrame, public DenseFrameBase
{
    friend class DenseFrame;

public:
    using FramePtr = std::shared_ptr< ProcessedDenseFrame >;

    static FramePtr create( const MapPtr &parentMap );

    void processDenseCloud();

protected:
    ProcessedDenseFrame( const MapPtr &parentMap );
};

class AdjacentFrame : public DoubleFrame, public ProcessedDoubleFrameBase
{
public:
    using FramePtr = std::shared_ptr< AdjacentFrame >;

    static FramePtr create();

    cv::Mat trackOptical();
    cv::Mat trackFeatures();

    double recoverPose();

    void setFrames( const ProcessedFramePtr &prevFrame, const ProcessedFramePtr &nextFrame );

    ProcessedFramePtr previousFrame() const;
    ProcessedFramePtr nextFrame() const;

    std::vector< AdjacentPoint > adjacentPoints() const;
    int adjacentPointsCount() const;

    std::vector< AdjacentFrame::ProcessedPointPtr > posePoints() const;
    int posePointsCount() const;

    std::vector< AdjacentFrame::ProcessedPointPtr > trackFramePoints() const;
    int trackFramePointsCount() const;

    std::vector< AdjacentFrame::ProcessedPointPtr > trackedPoints() const;
    int trackedPointsCount() const;

    void extractDescriptors();

    void createFramePoints( const size_t count );

protected:
    AdjacentFrame();

};

class StereoFrame : public StereoFrameBase
{
public:
    using FramePtr = std::shared_ptr< Frame >;
    using StereoFramePtr = std::shared_ptr< StereoFrame >;
    using ProcessedStereoFramePtr = std::shared_ptr< ProcessedStereoFrame >;

    static StereoFramePtr create( const MapPtr &parentMap );

    void setFrames( const FramePtr &leftFrame, const FramePtr &rightFrame );

    FramePtr leftFrame() const;
    FramePtr rightFrame() const;

    void replace( const ProcessedStereoFramePtr &frame );
    void replaceAndClean( const ProcessedStereoFramePtr &frame );

protected:
    StereoFrame( const MapPtr &parentMap );

private:
    void initialize();

};

class DenseFrame : public StereoFrame, public DenseFrameBase
{
public:
    using FramePtr = std::shared_ptr< DenseFrame >;
    using ProcessedDenseFramePtr = std::shared_ptr< ProcessedDenseFrame >;

    static FramePtr create( const MapPtr &parentMap );

    void replace( const ProcessedDenseFramePtr &frame );
    void replaceAndClean( const ProcessedDenseFramePtr &frame );

    std::list< ColorPoint3d > translatedPoints() const;

protected:
    DenseFrame( const MapPtr &parentMap );

    void replaceProcedure( const ProcessedDenseFramePtr &frame );

};

}
