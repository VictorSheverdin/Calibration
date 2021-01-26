#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "framepoint.h"
#include "mappoint.h"

#include "src/common/projectionmatrix.h"
#include "src/common/featureprocessor.h"
#include "src/common/stereoprocessor.h"
#include "src/common/matrix.h"

#include "alias.h"

namespace slam {

class World;
class Map;
class MapPoint;

// Базовый класс для всех классов кадра
class FrameBase
{
public:
    using ObjectPtr = std::shared_ptr< FrameBase >;
    using ObjectConstPtr = std::shared_ptr< const FrameBase >;

protected:
    FrameBase() = default;
    virtual ~FrameBase() = default;
};

class MonoFrame : public FrameBase, public std::enable_shared_from_this< MonoFrame >
{
    friend class DoubleFrame;

public:
    using ObjectPtr = std::shared_ptr< MonoFrame >;
    using ObjectConstPtr = std::shared_ptr< const MonoFrame >;

    virtual std::vector< MonoPointPtr > framePoints() const = 0;
    virtual size_t framePointsCount() const = 0;

    std::vector< cv::Point2f > points() const;

    std::vector< StereoPoint > stereoPoints() const;

protected:
    MonoFrame() = default;

};

class MonoKeyFrame : public virtual MonoFrame, public ProjectionMatrix
{
public:
    using ObjectPtr = std::shared_ptr< MonoKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const MonoKeyFrame >;

protected:
    MonoKeyFrame() = default;
};

class ProcessedFrame : public virtual MonoFrame
{
public:
    using ObjectPtr = std::shared_ptr< ProcessedFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcessedFrame >;

    virtual void removePoint( const MonoPointPtr &point ) = 0;

    MapPtr parentMap() const;
    WorldPtr parentWorld() const;

    void setImage( const StampedImage &image );
    const CvImage &image() const;

    const cv::Mat &mask() const;

    void clearImage();
    void clearMask();

    CvImage drawTracks() const;

    void cleanMapPoints();

    void removeExtraPoints( const size_t mapPointsCount, const size_t framePointsCount );

    std::vector< MonoPointPtr > posePoints() const;
    size_t posePointsCount() const;

    std::vector< MonoPointPtr > trackedPoints() const;
    size_t trackedPointsCount() const;

    void setImagePyramid( const std::vector< cv::Mat > &value );
    const std::vector< cv::Mat > &imagePyramid() const;

    const std::chrono::time_point< std::chrono::system_clock > &time() const;

    void setDesiredPointsCount( const size_t value );
    size_t desiredPointsCount() const;

    double pointsInterval() const;

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    MapPtrImpl m_parentMap;

    StampedImage m_image;

    cv::Mat m_mask;

    std::vector< cv::Mat > m_imagePyramid;

    static const int m_minConnectedPoints = 7;
    static const double m_densityFactor;

    size_t m_desiredPointsCount;

    ProcessedFrame( const MapPtr &parentMap );

private:
    void initialize();
};

class ProcessedKeyFrame : public virtual ProcessedFrame, public MonoKeyFrame
{
public:
    using ObjectPtr = std::shared_ptr< ProcessedKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcessedKeyFrame >;

protected:
    ProcessedKeyFrame( const MapPtr &parentMap );
};

class FlowFrame : public virtual ProcessedFrame
{    
    friend class FeaturePoint;
    friend class FlowKeyFrame;

public:
    using ObjectPtr = std::shared_ptr< FlowFrame >;
    using ObjectConstPtr = std::shared_ptr< const FlowFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void setImage( const StampedImage &image );

    void buildPyramid();
    void clearPyramid();

    virtual std::vector< MonoPointPtr > framePoints() const override;
    virtual size_t framePointsCount() const override;

    virtual void removePoint( const MonoPointPtr &point ) override;

    std::vector< FlowPointPtr > flowPoints() const;
    size_t flowPointsCount() const;

    void addFlowPoints( const std::vector< cv::Point2f > &vector );
    FlowPointPtr addFlowPoint( const cv::Point2f &point );

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

protected:
    FlowFrame( const MapPtr &parentMap );

    std::set< FlowPointPtr > m_flowPoints;

    std::vector< cv::KeyPoint > m_keyPoints;
    std::vector< cv::Scalar > m_colors;
    cv::Mat m_descriptors;

    std::map< size_t, FeaturePointPtr > m_featurePoints;

private:
    void initialize();
};

cv::Mat track( const std::shared_ptr< FlowFrame > &prevFrame, const std::shared_ptr< FlowFrame > &nextFrame );

class FlowKeyFrame : public FlowFrame, public ProcessedKeyFrame
{
public:
    using ObjectPtr = std::shared_ptr< FlowKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const FlowKeyFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void setImage( const StampedImage &image );

    void createMask();

    void extractPoints();

    void replace( const FlowFramePtr &frame );
    void replaceAndClean( const FlowFramePtr &frame );

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

protected:
    FlowKeyFrame( const MapPtr &parentMap );

private:
    void initialize();
};

/*
class FeatureFrame : public virtual ProcessedFrame
{
    friend class FeatureStereoFrame;
    friend class FeatureConsecutiveFrame;
    friend class FeaturePoint;

public:
    using ObjectPtr = std::shared_ptr< FeatureFrame >;
    using ObjectConstPtr = std::shared_ptr< const FeatureFrame >;

    virtual std::vector< MonoPointPtr > framePoints() const override;
    virtual size_t framePointsCount() const override;

    virtual void removePoint( const MonoPointPtr &point ) override;

    std::vector< FeaturePointPtr > featurePoints() const;

    FeaturePointPtr &featurePoint( const size_t index );
    const FeaturePointPtr &featurePoint( const size_t index ) const;

    static ObjectPtr create( const FeatureMapPtr &parentMap );

    void setImage( const StampedImage &image );

    void extractKeypoints();

    void createMask();

    void createFramePoints( const size_t count );

    const std::vector< cv::KeyPoint > &keyPoints() const;
    void setKeyPoints( const std::vector< cv::KeyPoint > &value );

    const cv::Mat &descriptors() const;
    void setDescriptors( const cv::Mat &value );

    std::shared_ptr< FeatureFrame > shared_from_this();
    std::shared_ptr< const FeatureFrame > shared_from_this() const;

protected:
    FeatureFrame( const FeatureMapPtr &parentMap );

    std::vector< cv::KeyPoint > m_keyPoints;
    std::vector< cv::Scalar > m_colors;
    cv::Mat m_descriptors;

    std::map< size_t, FeaturePointPtr > m_points;

    FeaturePointPtr createFramePoint( const size_t keyPointIndex );

    bool isFramePointExist( const size_t index ) const;
};
*/
class FinishedFrame : public virtual MonoFrame
{
public:
    using ObjectPtr = std::shared_ptr< FinishedFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinishedFrame >;

    static ObjectPtr create( const std::chrono::time_point< std::chrono::system_clock > time = std::chrono::system_clock::now() );

    void setTime( const std::chrono::time_point< std::chrono::system_clock > &value );
    const std::chrono::time_point< std::chrono::system_clock > &time() const;

    virtual std::vector< MonoPointPtr > framePoints() const override;
    virtual size_t framePointsCount() const override;

    FinishedFramePointPtr createFramePoint( const cv::Point2f &point, const cv::Scalar &color );

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    void replace( const ProcessedFramePtr &frame );
    void replaceAndClean( const ProcessedFramePtr &frame );

protected:
    FinishedFrame( const std::chrono::time_point< std::chrono::system_clock > &time );

    std::chrono::time_point< std::chrono::system_clock > m_time;

    std::list< FinishedFramePointPtr > m_points;

};

class FinishedKeyFrame : public FinishedFrame, public MonoKeyFrame
{
public:
    using ObjectPtr = std::shared_ptr< FinishedKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinishedKeyFrame >;

    static ObjectPtr create( const std::chrono::time_point< std::chrono::system_clock > time = std::chrono::system_clock::now() );

    void replace( const ProcessedKeyFramePtr &frame );
    void replaceAndClean( const ProcessedKeyFramePtr &frame );

protected:
    FinishedKeyFrame( const std::chrono::time_point< std::chrono::system_clock > &time );

};

class DoubleFrame : public FrameBase
{
public:
    using ObjectPtr = std::shared_ptr< DoubleFrame >;
    using ObjectConstPtr = std::shared_ptr< const DoubleFrame >;

    void setFrame1( const MonoFramePtr &frame );
    void setFrame2( const MonoFramePtr &frame );
    void setFrames( const MonoFramePtr &frame1, const MonoFramePtr &frame2 );

    MonoFramePtr frame1() const;
    MonoFramePtr frame2() const;

protected:
    DoubleFrame();

    MonoFramePtr m_frame1;
    MonoFramePtr m_frame2;
};

class StereoFrame : public DoubleFrame
{
public:
    using ObjectPtr = std::shared_ptr< StereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const StereoFrame >;

    void setLeftFrame( const MonoFramePtr &frame );
    void setRightFrame( const MonoFramePtr &frame );
    void setFrames( const MonoFramePtr &leftFrame, const MonoFramePtr &rightFrame );

    std::vector< StereoPoint > stereoPoints() const;
    size_t stereoPointsCount() const;

    MonoFramePtr leftFrame() const;
    MonoFramePtr rightFrame() const;

    MapPtr parentMap() const;
    WorldPtr parentWorld() const;

protected:
    using MapPtrImpl = std::weak_ptr< Map >;

    StereoFrame( const MapPtr &parentMap );

    MapPtrImpl m_parentMap;
};

class ProcessedStereoFrame : public virtual StereoFrame
{
public:
    using ObjectPtr = std::shared_ptr< ProcessedStereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcessedStereoFrame >;

    ProcessedFramePtr leftFrame() const;
    ProcessedFramePtr rightFrame() const;

    void clearImages();

    void cleanMapPoints();

    CvImage drawStereoCorrespondences() const;
    CvImage drawTracks() const;

    void removeExtraPoints( const size_t mapPointsCount, const size_t framePointsCount );

protected:
    ProcessedStereoFrame( const MapPtr &parentMap );
};

class StereoKeyFrame : public virtual StereoFrame
{
public:
    using ObjectPtr = std::shared_ptr< StereoKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const StereoKeyFrame >;

    MonoKeyFramePtr leftFrame() const;
    MonoKeyFramePtr rightFrame() const;

    void setProjectionMatrix( const ProjectionMatrix &matrix1, const ProjectionMatrix &matrix2 );

    void setProjectionMatrix( const StereoCameraMatrix &matrix );
    StereoCameraMatrix projectionMatrix() const;

    double bf() const;

    void setLeftSe3Pose( g2o::SE3Quat &pose );

    cv::Point3f center() const;

    int triangulatePoints();

protected:
    StereoKeyFrame( const MapPtr &parentMap );
};

class ProcessedStereoKeyFrame : public virtual ProcessedStereoFrame, public virtual StereoKeyFrame
{
public:
    using ObjectPtr = std::shared_ptr< ProcessedStereoKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcessedStereoKeyFrame >;

    ProcessedKeyFramePtr leftFrame() const;
    ProcessedKeyFramePtr rightFrame() const;

protected:
    ProcessedStereoKeyFrame( const MapPtr &parentMap );
};

class FlowStereoFrame : public virtual ProcessedStereoFrame
{
public:
    using ObjectPtr = std::shared_ptr< FlowStereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const FlowStereoFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void setLeftImage( const StampedImage &image );
    void setRightImage( const StampedImage &image );

    void setImage( const StampedImage &leftImage, const StampedImage &rightImage );

    void buildPyramid();
    void clearPyramid();

    FlowFramePtr leftFrame() const;
    FlowFramePtr rightFrame() const;

    cv::Mat match();

protected:
    FlowStereoFrame( const MapPtr &parentMap );

};

class FlowStereoKeyFrame : public virtual ProcessedStereoKeyFrame, public FlowStereoFrame
{
public:
    using ObjectPtr = std::shared_ptr< FlowStereoKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const FlowStereoKeyFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void extractPoints();

    FlowKeyFramePtr leftFrame() const;
    FlowKeyFramePtr rightFrame() const;

    void replace( const FlowStereoFramePtr &frame );
    void replaceAndClean( const FlowStereoFramePtr &frame );

protected:
    FlowStereoKeyFrame( const MapPtr &parentMap );
};

class DenseFrame : public virtual StereoKeyFrame
{
public:
    void setPoints( const std::list< ColorPoint3d > &list );
    const std::list< ColorPoint3d > &points() const;

protected:
    DenseFrame( const MapPtr &parentMap );

    using OptimizationGrid = std::map< int, std::map< int, std::map< int, std::list< ColorPoint3d > > > >;

    std::list< ColorPoint3d > m_points;

    OptimizationGrid m_optimizationGrid;

    void createOptimizationGrid();
    void setOptimizationGrid( const OptimizationGrid &grid );

};

class ProcessedDenseFrame : public virtual ProcessedStereoKeyFrame, public DenseFrame
{
    friend class FinishedDenseFrame;

public:
    using ObjectPtr = std::shared_ptr< ProcessedDenseFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcessedDenseFrame >;

    void processDenseCloud();

protected:
    ProcessedDenseFrame( const MapPtr &parentMap );
};

class FlowDenseFrame : public FlowStereoKeyFrame, public ProcessedDenseFrame
{
public:
    using ObjectPtr = std::shared_ptr< FlowDenseFrame >;
    using ObjectConstPtr = std::shared_ptr< const FlowDenseFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

protected:
    FlowDenseFrame( const MapPtr &parentMap );

};

class ConsecutiveFrame : public DoubleFrame
{
public:
    using ObjectPtr = std::shared_ptr< ConsecutiveFrame >;
    using ObjectConstPtr = std::shared_ptr< const ConsecutiveFrame >;

    MonoFramePtr startFrame() const;
    MonoFramePtr endFrame() const;

    std::vector< ConsecutivePoint > points() const;
    std::vector< ConsecutivePoint > mapPoints() const;

    double averageMapPointsDisplacement() const;

protected:
    ConsecutiveFrame( const MonoFramePtr &startFrame, const MonoFramePtr &endFrame );

};

class FlowConsecutiveFrame : public ConsecutiveFrame
{
public:
    using ObjectPtr = std::shared_ptr< FlowConsecutiveFrame >;
    using ObjectConstPtr = std::shared_ptr< const FlowConsecutiveFrame >;

    FlowConsecutiveFrame( const FlowFramePtr &startFrame, const FlowFramePtr &endFrame );

    FlowFramePtr startFrame() const;
    FlowFramePtr endFrame() const;
};

class RecoverPoseFrame : public ConsecutiveFrame
{
public:
    using ObjectPtr = std::shared_ptr< RecoverPoseFrame >;
    using ObjectConstPtr = std::shared_ptr< const RecoverPoseFrame >;

    RecoverPoseFrame( const ProcessedKeyFramePtr &startFrame, const ProcessedFramePtr &endFrame );

    ProcessedKeyFramePtr startFrame() const;
    ProcessedFramePtr endFrame() const;

    double recoverPose( ProjectionMatrix *result );
};

class ConsecutiveKeyFrame : public ConsecutiveFrame
{
public:
    using ObjectPtr = std::shared_ptr< ConsecutiveKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const ConsecutiveKeyFrame >;

    ConsecutiveKeyFrame( const ProcessedKeyFramePtr &startFrame, const ProcessedKeyFramePtr &endFrame );

    ProcessedKeyFramePtr startFrame() const;
    ProcessedKeyFramePtr endFrame() const;

    int triangulatePoints();

    double distance() const;

};

class FinishedStereoFrame : public StereoFrame
{
public:
    using ObjectPtr = std::shared_ptr< FinishedStereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinishedStereoFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void setFrames( const FinishedFramePtr &leftFrame, const FinishedFramePtr &rightFrame );

    FinishedFramePtr leftFrame() const;
    FinishedFramePtr rightFrame() const;

    void replace( const ProcessedStereoFramePtr &frame );
    void replaceAndClean( const ProcessedStereoFramePtr &frame );

protected:
    FinishedStereoFrame( const MapPtr &parentMap );
};

class FinishedStereoKeyFrame : public virtual StereoKeyFrame
{
public:
    using ObjectPtr = std::shared_ptr< FinishedStereoKeyFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinishedStereoKeyFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void setFrames( const FinishedKeyFramePtr &leftFrame, const FinishedKeyFramePtr &rightFrame );

    FinishedKeyFramePtr leftFrame() const;
    FinishedKeyFramePtr rightFrame() const;

    void replace( const ProcessedStereoKeyFramePtr &frame );
    void replaceAndClean( const ProcessedStereoKeyFramePtr &frame );

protected:
    FinishedStereoKeyFrame( const MapPtr &parentMap );
};

class FinishedDenseFrame : public FinishedStereoKeyFrame, public DenseFrame
{
public:
    using ObjectPtr = std::shared_ptr< FinishedDenseFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinishedDenseFrame >;

    static ObjectPtr create( const MapPtr &parentMap );

    void replace( const ProcessedDenseFramePtr &frame );
    void replaceAndClean( const ProcessedDenseFramePtr &frame );

    std::list< ColorPoint3d > translatedPoints() const;

protected:
    FinishedDenseFrame( const MapPtr &parentMap );

    void replaceProcedure( const ProcessedDenseFramePtr &frame );

    static const double m_maximumLenght;
};

}
