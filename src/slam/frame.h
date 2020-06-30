#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "framepoint.h"
#include "mappoint.h"

#include "src/common/projectionmatrix.h"
#include "src/common/featureprocessor.h"
#include "src/common/stereoprocessor.h"
#include "src/common/matrix.h"

#include <g2o/types/slam3d/se3quat.h>

namespace slam {

class World;
class Map;
class FlowMap;
class FeatureMap;
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

    cv::Point3d point() const;

    void setSe3Pose( const g2o::SE3Quat &pose );
    g2o::SE3Quat se3Pose() const;

protected:
    MonoFrame();

private:
    void initialize();

};

class ProcessedFrame : public MonoFrame
{
public:
    using MapPtr = std::shared_ptr< Map >;
    using WorldPtr = std::shared_ptr< World >;

    virtual std::vector< cv::Point2f > extractedPoints() const = 0;
    size_t extractedPointsCount() const;

    MapPtr parentMap() const;
    WorldPtr parentWorld() const;

    const CvImage &image() const;
    void clearImage();

    CvImage drawExtractedPoints() const;
    CvImage drawTracks() const;

    int triangulatePoints();

    void cleanMapPoints();

    std::vector< PointPtr > posePoints() const;
    size_t posePointsCount() const;

    std::vector< PointPtr > spatialPoints() const;
    size_t spatialPointsCount() const;

    std::vector< PointPtr > trackFramePoints() const;
    size_t trackFramePointsCount() const;

    std::vector< PointPtr > trackedPoints() const;
    size_t trackedPointsCount() const;

    void setImagePyramid( const std::vector< cv::Mat > &value );
    const std::vector< cv::Mat > &imagePyramid() const;

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    MapPtrImpl m_parentMap;

    CvImage m_image;

    std::vector< cv::Mat > m_imagePyramid;

    static const int m_minConnectedPoints = 7;

    static const double m_minPointsDistance;

    ProcessedFrame( const MapPtr &parentMap );

    void load( const CvImage &image );

};

class FlowFrame : public ProcessedFrame, public std::enable_shared_from_this< FlowFrame >
{
public:
    using ObjectPtr = std::shared_ptr< FlowFrame >;
    using FlowMapPtr = std::shared_ptr< FlowMap >;
    using FlowPointPtr = std::shared_ptr< FlowPoint >;

    static ObjectPtr create( const FlowMapPtr &parentMap );

    void load( const CvImage &image );

    void buildPyramid();
    void extractPoints();

    virtual std::vector< PointPtr > framePoints() const override;
    virtual std::vector< cv::Point2f > extractedPoints() const override;

    std::vector< FlowPointPtr > &flowPoints();
    const std::vector< FlowPointPtr > &flowPoints() const;

    FlowPointPtr framePoint( const cv::Point2f &point ) const;

    void createFramePoints( const size_t count );
    FlowPointPtr addFramePoint( const cv::Point2f &point );

    void setExtractedPoints( const std::vector< cv::Point2f > &value );

protected:
    FlowFrame( const FlowMapPtr &parentMap );

    std::vector< cv::Point2f > m_extractedPoints;

    std::vector< FlowPointPtr > m_points;

    size_t m_usePointsCount;

    matrix< FlowPointPtr > m_searchMatrix;

    FlowPointPtr createFramePoint( const size_t keyPointIndex );

private:
    void initialize();

};

class FeatureFrame : public ProcessedFrame, public std::enable_shared_from_this< FeatureFrame >
{
    friend class FeatureStereoFrame;
    friend class FeatureConsecutiveFrame;
    friend class FeaturePoint;

public:
    using ObjectPtr = std::shared_ptr< FeatureFrame >;
    using FeatureMapPtr = std::shared_ptr< FeatureMap >;
    using MapPointPtr = std::shared_ptr< MapPoint >;

    using FeaturePointPtr = std::shared_ptr< FeaturePoint >;

    virtual std::vector< PointPtr > framePoints() const override;
    virtual std::vector< cv::Point2f > extractedPoints() const override;

    std::vector< FeaturePointPtr > featurePoints() const;

    FeaturePointPtr &featurePoint( const size_t index );
    const FeaturePointPtr &featurePoint( const size_t index ) const;

    static ObjectPtr create( const FeatureMapPtr &parentMap );

    void load( const CvImage &image );

    void createFramePoints( const size_t count );

    const std::vector< cv::KeyPoint > &keyPoints() const;
    void setKeyPoints( const std::vector< cv::KeyPoint > &value );

    const cv::Mat &descriptors() const;
    void setDescriptors( const cv::Mat &value );

protected:
    FeatureFrame( const FeatureMapPtr &parentMap );

    std::vector< cv::KeyPoint > m_keyPoints;
    std::vector< cv::Scalar > m_colors;
    cv::Mat m_descriptors;

    std::map< size_t, FeaturePointPtr > m_points;

    FeaturePointPtr createFramePoint( const size_t keyPointIndex );

    bool isFramePointExist( const size_t index ) const;

private:
    void initialize();

};

class Frame : public MonoFrame, public std::enable_shared_from_this< Frame >
{
public:
    using ObjectPtr = std::shared_ptr< Frame >;
    using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;

    using FramePointPtr = std::shared_ptr< FramePoint >;

    virtual std::vector< PointPtr > framePoints() const override;

    static ObjectPtr create();

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
    friend class FeatureMap;

public:
    using MapPtr = std::shared_ptr< Map >;
    using WorldPtr = std::shared_ptr< World >;

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

class StereoFrameBase : public DoubleFrame
{
public:
    using FramePtr = std::shared_ptr< StereoFrameBase >;

    void setFrames( const MonoFramePtr &leftFrame, const MonoFramePtr &rightFrame );

    void setProjectionMatrix( const StereoCameraMatrix &matrix );
    StereoCameraMatrix projectionMatrix() const;

    double bf() const;

    MonoFramePtr leftFrame() const;
    MonoFramePtr rightFrame() const;

    void setLeftSe3Pose( g2o::SE3Quat &pose );

    MapPtr parentMap() const;
    WorldPtr parentWorld() const;

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    StereoFrameBase( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;

private:
};

class ProcessedStereoFrame : public StereoFrameBase
{
public:
    using MapPointPtr = std::shared_ptr< MapPoint >;

    using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;

    ProcessedFramePtr leftFrame() const;
    ProcessedFramePtr rightFrame() const;

    void clearImages();

    CvImage drawExtractedPoints() const;
    CvImage drawStereoCorrespondences() const;
    CvImage drawTracks() const;

    std::vector< StereoPoint > stereoPoints() const;
    size_t stereoPointsCount() const;

    int triangulatePoints();

    void cleanMapPoints();

protected:
    ProcessedStereoFrame( const MapPtr &parentMap );

};

class FlowStereoFrame : public ProcessedStereoFrame
{
public:
    using ObjectPtr = std::shared_ptr< FlowStereoFrame >;
    using MapPtr = std::shared_ptr< FlowMap >;
    using FlowFramePtr = std::shared_ptr< FlowFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void load( const CvImage &image1, const CvImage &image2 );

    void buildPyramid();
    void extractPoints();

    FlowFramePtr leftFrame() const;
    FlowFramePtr rightFrame() const;

    MapPtr parentMap() const;

    cv::Mat match();

protected:
    FlowStereoFrame( const MapPtr &parentMap );

};

class FeatureStereoFrame : public ProcessedStereoFrame
{
public:
    using ObjectPtr = std::shared_ptr< FeatureStereoFrame >;
    using MapPtr = std::shared_ptr< FeatureMap >;
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void load( const CvImage &image1, const CvImage &image2 );

    FeatureFramePtr leftFrame() const;
    FeatureFramePtr rightFrame() const;

    MapPtr parentMap() const;

    cv::Mat match();

protected:
    FeatureStereoFrame( const MapPtr &parentMap );

};

class DenseFrameBase
{
    friend class DenseFrame;

public:
    virtual ~DenseFrameBase();

    void setPoints( const std::list< ColorPoint3d > &list );
    const std::list< ColorPoint3d > &points() const;

protected:
    DenseFrameBase();

    using OptimizationGrid = std::map< int, std::map< int, std::map< int, std::list< ColorPoint3d > > > >;

    std::list< ColorPoint3d > m_points;

    OptimizationGrid m_optimizationGrid;

    void createOptimizationGrid();
    void setOptimizationGrid( const OptimizationGrid &grid );

private:
    void initialize();

};

template < class ProcessedFrameType >
class ProcessedDenseFrame : public ProcessedFrameType, public DenseFrameBase
{
    friend class DenseFrame;

public:
    using ObjectPtr = std::shared_ptr< ProcessedDenseFrame< ProcessedFrameType > >;
    using MapPtr = typename ProcessedFrameType::MapPtr;

    static ObjectPtr create( const MapPtr &parentMap );

    void processDenseCloud();

protected:
    ProcessedDenseFrame( const MapPtr &parentMap );

};

using FlowDenseFrame = ProcessedDenseFrame< FlowStereoFrame >;
using FeatureDenseFrame = ProcessedDenseFrame< FeatureStereoFrame >;

class ConsecutiveFrame : public DoubleFrame
{
public:
    using MapPtr = std::shared_ptr< Map >;
    using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;

    ProcessedFramePtr previousFrame() const;
    ProcessedFramePtr nextFrame() const;

    double recoverPose();

    MapPtr parentMap() const;
    WorldPtr parentWorld() const;

    std::vector< AdjacentPoint > adjacentPoints() const;
    size_t adjacentPointsCount() const;

    std::vector< MonoPointPtr > posePoints() const;
    size_t posePointsCount() const;

    std::vector< MonoPointPtr > spatialPoints() const;
    size_t spatialPointsCount() const;

    std::vector< MonoPointPtr > trackFramePoints() const;
    size_t trackFramePointsCount() const;

    std::vector< MonoPointPtr > trackedPoints() const;
    size_t trackedPointsCount() const;

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    ConsecutiveFrame( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;

};

class FlowConsecutiveFrame : public ConsecutiveFrame
{
public:
    using ObjectPtr = std::shared_ptr< FlowConsecutiveFrame >;
    using FlowMapPtr = std::shared_ptr< FlowMap >;
    using FlowFramePtr = std::shared_ptr< FlowFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    cv::Mat track();

    FlowFramePtr previousFrame() const;
    FlowFramePtr nextFrame() const;

    void createFramePoints( const size_t count );

protected:
    FlowConsecutiveFrame( const MapPtr &parentMap );

};

class FeatureConsecutiveFrame : public ConsecutiveFrame
{
public:
    using ObjectPtr = std::shared_ptr< FeatureConsecutiveFrame >;
    using FeatureMapPtr = std::shared_ptr< FeatureMap >;
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    cv::Mat track();

    FeatureFramePtr previousFrame() const;
    FeatureFramePtr nextFrame() const;

    void createFramePoints( const size_t count );

protected:
    FeatureConsecutiveFrame( const MapPtr &parentMap );

};

class StereoFrame : public StereoFrameBase
{
public:
    using ObjectPtr = std::shared_ptr< StereoFrame >;
    using FramePtr = std::shared_ptr< Frame >;
    using ProcessedStereoFramePtr = std::shared_ptr< ProcessedStereoFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

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
    using ObjectPtr = std::shared_ptr< DenseFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    template < class DENSE_FRAME_TYPE >
    void replace( const std::shared_ptr< DENSE_FRAME_TYPE > &frame );
    template < class DENSE_FRAME_TYPE >
    void replaceAndClean( const std::shared_ptr< DENSE_FRAME_TYPE > &frame );

    std::list< ColorPoint3d > translatedPoints() const;

protected:
    DenseFrame( const MapPtr &parentMap );

    template < class DENSE_FRAME_TYPE >
    void replaceProcedure( const std::shared_ptr< DENSE_FRAME_TYPE > &frame );

    static const double m_maximumLenght;\

};

#include "frame.inl"

}
