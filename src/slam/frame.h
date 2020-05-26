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

class World;
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
    using MapPtr = std::shared_ptr< FeatureMap >;
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
    int posePointsCount() const;

    std::vector< PointPtr > trackFramePoints() const;
    int trackFramePointsCount() const;

    std::vector< PointPtr > trackedPoints() const;
    int trackedPointsCount() const;

    void setImagePyramid( const std::vector< cv::Mat > &value );
    const std::vector< cv::Mat > &imagePyramid() const;

protected:
    using MapPtrImpl = std::weak_ptr< FeatureMap >;

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
    using FramePtr = std::shared_ptr< FlowFrame >;
    using FlowPointPtr = std::shared_ptr< FlowPoint >;

    static FramePtr create( const MapPtr &parentMap );

    virtual std::vector< PointPtr > framePoints() const override;
    virtual std::vector< cv::Point2f > extractedPoints() const override;

    std::list< FlowPointPtr > &flowPoints();
    const std::list< FlowPointPtr > &flowPoints() const;

    void extractPoints();

    void createFramePoints( const size_t count );

protected:
    FlowFrame( const MapPtr &parentMap );

    std::vector< cv::Point2f > m_extractedPoints;

    std::list< FlowPointPtr > m_points;

    size_t m_usePointsCount;

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
    using MapPointPtr = std::shared_ptr< MapPoint >;
    using FramePtr = std::shared_ptr< FeatureFrame >;

    using FeaturePointPtr = std::shared_ptr< FeaturePoint >;

    virtual std::vector< PointPtr > framePoints() const override;
    virtual std::vector< cv::Point2f > extractedPoints() const override;

    std::vector< FeaturePointPtr > featurePoints() const;

    FeaturePointPtr &featurePoint( const size_t index );
    const FeaturePointPtr &featurePoint( const size_t index ) const;

    static FramePtr create( const MapPtr &parentMap );

    void load( const CvImage &image );

    void createFramePoints( const size_t count );

    const std::vector< cv::KeyPoint > &keyPoints() const;
    void setKeyPoints( const std::vector<cv::KeyPoint> &value );

    const cv::Mat &descriptors() const;
    void setDescriptors( const cv::Mat &value );

    const cv::Mat &searchMatrix() const;
    void setSearchMatrix( const cv::Mat &value );

protected:
    FeatureFrame( const MapPtr &parentMap );

    std::vector< cv::KeyPoint > m_keyPoints;
    std::vector< cv::Scalar > m_colors;
    cv::Mat m_descriptors;

    std::map< size_t, FeaturePointPtr > m_points;

    cv::Mat m_searchMatrix;

    FeaturePointPtr createFramePoint( const size_t keyPointIndex );

    bool isFramePointExist( const size_t index ) const;

private:
    void initialize();

};

class Frame : public MonoFrame, public std::enable_shared_from_this< Frame >
{
public:
    using FramePtr = std::shared_ptr< Frame >;
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;

    using FramePointPtr = std::shared_ptr< FramePoint >;

    virtual std::vector< PointPtr > framePoints() const override;

    static FramePtr create();

    void replace( const FeatureFramePtr &frame );
    void replaceAndClean( const FeatureFramePtr &frame );

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
    using MapPtr = std::shared_ptr< FeatureMap >;
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
    using MapPtrImpl = std::weak_ptr< FeatureMap >;

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
    int stereoPointsCount() const;

    int triangulatePoints();

protected:
    ProcessedStereoFrame( const MapPtr &parentMap );
};

class FlowStereoFrame : public ProcessedStereoFrame
{
public:
    using FramePtr = std::shared_ptr< FlowStereoFrame >;

    static FramePtr create( const MapPtr &parentMap );

protected:
    FlowStereoFrame( const MapPtr &parentMap );

};

class FeatureStereoFrame : public ProcessedStereoFrame
{
public:
    using FramePtr = std::shared_ptr< FeatureStereoFrame >;
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;

    static FramePtr create( const MapPtr &parentMap );

    void load( const CvImage &image1, const CvImage &image2 );

    FeatureFramePtr leftFrame() const;
    FeatureFramePtr rightFrame() const;

    cv::Mat match();

    void cleanMapPoints();

protected:
    FeatureStereoFrame( const MapPtr &parentMap );

    static const double m_minXDistasnce;
    static const double m_maxYDistasnce;

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
    using FramePtr = std::shared_ptr< ProcessedDenseFrame< ProcessedFrameType > >;
    using MapPtr = typename ProcessedFrameType::MapPtr;

    static FramePtr create( const MapPtr &parentMap );

    void processDenseCloud();

protected:
    ProcessedDenseFrame( const MapPtr &parentMap );

};

using FeatureDenseFrame = ProcessedDenseFrame< FeatureStereoFrame >;
using FlowDenseFrame = ProcessedDenseFrame< FlowStereoFrame >;

class ConsecutiveFrame : public DoubleFrame
{
public:
    using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;

    ProcessedFramePtr previousFrame() const;
    ProcessedFramePtr nextFrame() const;

protected:
    ConsecutiveFrame() = default;

};

class FeatureConsecutiveFrame : public ConsecutiveFrame
{
public:
    using FramePtr = std::shared_ptr< FeatureConsecutiveFrame >;
    using FeatureFramePtr = std::shared_ptr< FeatureFrame >;

    static FramePtr create( const MapPtr &parentMap );

    cv::Mat track();

    double recoverPose();

    FeatureFramePtr previousFrame() const;
    FeatureFramePtr nextFrame() const;

    std::vector< AdjacentPoint > adjacentPoints() const;
    int adjacentPointsCount() const;

    std::vector< FeatureConsecutiveFrame::MonoPointPtr > posePoints() const;
    int posePointsCount() const;

    std::vector< MonoPointPtr > trackFramePoints() const;
    int trackFramePointsCount() const;

    std::vector< FeatureConsecutiveFrame::MonoPointPtr > trackedPoints() const;
    int trackedPointsCount() const;

    void createFramePoints( const size_t count );

    MapPtr parentMap() const;
    WorldPtr parentWorld() const;

protected:
    using MapPtrImpl = std::weak_ptr< FeatureMap >;

    FeatureConsecutiveFrame( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;


};

class StereoFrame : public StereoFrameBase
{
public:
    using FramePtr = std::shared_ptr< Frame >;
    using StereoFramePtr = std::shared_ptr< StereoFrame >;
    using ProcessedStereoFramePtr = std::shared_ptr< FeatureStereoFrame >;

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
    using ProcessedDenseFramePtr = std::shared_ptr< FeatureDenseFrame >;

    static FramePtr create( const MapPtr &parentMap );

    void replace( const ProcessedDenseFramePtr &frame );
    void replaceAndClean( const ProcessedDenseFramePtr &frame );

    std::list< ColorPoint3d > translatedPoints() const;

protected:
    DenseFrame( const MapPtr &parentMap );

    void replaceProcedure( const ProcessedDenseFramePtr &frame );

    static const double m_maximumLenght;\


};

#include "frame.inl"

}
