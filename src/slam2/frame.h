#pragma once

#include <vector>
#include <memory>

#include "src/common/supportclasses.h"

#include "src/common/colorpoint.h"
#include "src/common/image.h"
#include "src/common/projectionmatrix.h"

#include "alias.h"

namespace slam2 {

class StereoCameraMatrix;
class StereoDistorsionCoefficients;


class Frame : public std::enable_shared_from_this< Frame >, protected Parent_Shared_Ptr< StereoFrame >
{
public:
    using ObjectClass = Frame;
    using ObjectPtr = std::shared_ptr< Frame >;
    using ObjectConstPtr = std::shared_ptr< const Frame >;

    virtual ~Frame() = default;

    std::shared_ptr< StereoFrame > parentStereoFrame() const;
    std::shared_ptr< Map > parentMap() const;
    std::shared_ptr< System > parentSystem() const;

    TrackPtr createTrack();

protected:
    Frame( const StereoFramePtr &parent );

    std::vector< TrackPtr > _tracks;
};

class FinalFrame : public Frame
{
public:
    using ObjectClass = FinalFrame;
    using ParentClass = Frame;
    using ObjectPtr = std::shared_ptr< FinalFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinalFrame >;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    void setTime( const std::chrono::time_point< std::chrono::system_clock > &time );
    const std::chrono::time_point< std::chrono::system_clock > &time() const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

protected:
    FinalFrame( const StereoFramePtr &parent );

    std::vector< FramePointPtr > _points;

    cv::Mat _cameraMatrix;

    std::chrono::time_point< std::chrono::system_clock > _time;

private:
    void initialize();

};

class ProcFrame : public FinalFrame
{
    friend class ProcStereoFrame;
    friend class CPUFlowTracker;
    friend class GPUFlowTracker;
    friend class SiftTracker;
    friend class OrbTracker;
    friend class AKazeTracker;
    friend class SuperGlueTracker;
public:
    using ObjectClass = ProcFrame;
    using ParentClass = Frame;
    using ObjectPtr = std::shared_ptr< ProcFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcFrame >;

    static ObjectPtr create( const StereoFramePtr &parent );

    void load( const StampedImage &image );

    const cv::Point2f &cornerPoint( const size_t index ) const;
    const cv::Point2f &undistortedCornerPoint( const size_t index ) const;

    const cv::KeyPoint &keyPoint( const size_t index ) const;
    const cv::KeyPoint &undistortedKeyPoint( const size_t index ) const;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    void setDistorsionCoefficients( const cv::Mat &value );
    const cv::Mat &distorsionCoefficients() const;

    const StampedImage &image() const;
    cv::Mat mask() const;

    FlowPointPtr createFlowPoint( const size_t index );
    FeaturePointPtr createFeaturePoint( const size_t index );

    FlowPointPtr flowPoint( const size_t index ) const;
    FeaturePointPtr featurePoint( const size_t index ) const;

    CvImage drawPoints() const;
    CvImage drawTracks() const;

    size_t tracksCount() const;

    void clearMemory();

protected:
    ProcFrame( const StereoFramePtr &parent );

    StampedImage _image;

    std::vector< cv::Mat > _imagePyramid;

    cv::Mat _distCoefficients;

    std::vector< cv::Point2f > _cornerPoints;
    std::vector< cv::Point2f > _undistCornerPoints;

    std::map< size_t, FlowPointPtr > _flowPoints;

    std::vector< cv::KeyPoint > _keyPoints;
    std::vector< cv::KeyPoint > _undistKeyPoints;
    cv::Mat _descriptors;

    std::map< size_t, FeaturePointPtr > _featurePoints;

    size_t addCornerPoint( const cv::Point2f &point );
    size_t addCornerPoints( const std::vector< cv::Point2f > &points );
    const std::vector< cv::Point2f > &cornerPoints() const;
    const std::vector< cv::Point2f > &undistortedCornerPoints() const;

    void setKeyPoints( const std::vector< cv::KeyPoint > &value );
    const std::vector< cv::KeyPoint > &keyPoints() const;
    const std::vector< cv::KeyPoint > &undistortedKeyPoints() const;

    void setImagePyramid( const std::vector< cv::Mat > &value );
    const std::vector< cv::Mat > &imagePyramid() const;

    void setDescriptors( const cv::Mat &value );
    const cv::Mat &descriptors() const;

    size_t extractionCornersCount() const;

    void undistortPoints( const std::vector< cv::Point2f > &sourcePoints, std::vector< cv::Point2f > *undistortedPoints ) const;

};

class DoubleFrame : public std::enable_shared_from_this< DoubleFrame >, protected Parent_Shared_Ptr< Map >
{
public:
    using ObjectClass = DoubleFrame;
    using ObjectPtr = std::shared_ptr< DoubleFrame >;
    using ObjectConstPtr = std::shared_ptr< const DoubleFrame >;

    virtual ~DoubleFrame() = default;

    void set( const FramePtr &frame1, const FramePtr &frame2 );

    const FramePtr &frame1() const;
    const FramePtr &frame2() const;

    std::shared_ptr< Map > parentMap() const;
    std::shared_ptr< System > parentSystem() const;

protected:
    DoubleFrame( const MapPtr &parent );

    FramePtr _frame1;
    FramePtr _frame2;

};

class StereoFrame : public DoubleFrame
{
public:
    using ObjectClass = StereoFrame;
    using ParentClass = DoubleFrame;
    using ObjectPtr = std::shared_ptr< StereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const StereoFrame >;

    virtual ~StereoFrame() = default;

    void set( const FramePtr &leftFrame, const FramePtr &rightFrame );

    const FramePtr &leftFrame() const;
    const FramePtr &rightFrame() const;

    void setRotation( const cv::Mat &value );
    const cv::Mat &rotation() const;

    void setTranslation( const cv::Mat &value );
    const cv::Mat &translation() const;

    void setRightRotation( const cv::Mat &value );
    const cv::Mat &rightRotation() const;

    void setRightTranslation( const cv::Mat &value );
    const cv::Mat &rightTranslation() const;

protected:
    StereoFrame( const MapPtr &parent );

    cv::Mat _rotation;
    cv::Mat _translation;

    cv::Mat _rightRotation;
    cv::Mat _rightTranslation;

private:
    void initialize();

};

class FinalStereoFrame : public StereoFrame
{
public:
    using ObjectClass = FinalStereoFrame;
    using ParentClass = StereoFrame;
    using ObjectPtr = std::shared_ptr< FinalStereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinalStereoFrame >;

    static ObjectPtr create( const MapPtr &parent );

    FinalFramePtr leftFrame() const;
    FinalFramePtr rightFrame() const;

    void setCameraMatrices( const StereoCameraMatrix &value );

    ProjectionMatrix leftProjectionMatrix() const;
    ProjectionMatrix rightProjectionMatrix() const;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

protected:
    FinalStereoFrame( const MapPtr &parent );

};

class ProcStereoFrame : public FinalStereoFrame
{
    friend class CPUFlowTracker;
    friend class GPUFlowTracker;
    friend class SiftTracker;
    friend class OrbTracker;
    friend class AKazeTracker;
    friend class SuperGlueTracker;
public:
    using ObjectClass = ProcStereoFrame;
    using ParentClass = FinalStereoFrame;
    using ObjectPtr = std::shared_ptr< ProcStereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcStereoFrame >;

    static ObjectPtr create( const MapPtr &parent );

    void load( const StampedStereoImage &image );

    void prepareFrame();
    void extract();
    void match();

    size_t triangulatePoints();

    ProcFramePtr leftFrame() const;
    ProcFramePtr rightFrame() const;

    void setDistorsionCoefficients( const StereoDistorsionCoefficients &value );

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    FlowStereoPointPtr createFlowPoint( const size_t leftIndex, const size_t rightIndex );
    FeatureStereoPointPtr createFeaturePoint( const size_t leftIndex , const size_t rightIndex );

    CvImage drawPoints() const;
    CvImage drawTracks() const;
    CvImage drawStereo() const;

    std::vector< ColorPoint3d > sparseCloud() const;

    void clearMemory();

protected:
    ProcStereoFrame( const MapPtr &parent );

    void setImagePyramid( const std::vector< cv::Mat > &leftPyramid, const std::vector< cv::Mat > &rightPyramid );

    void setKeyPoints( const std::vector< cv::KeyPoint > &left, const std::vector< cv::KeyPoint > &right );
    void setDescriptors( const cv::Mat &left, const cv::Mat &right );

    std::vector< ProcStereoPointPtr > _points;

};

class ConsecutiveFrames : public DoubleFrame
{
public:
    using ObjectClass = ConsecutiveFrames;
    using ParentClass = DoubleFrame;
    using ObjectPtr = std::shared_ptr< ConsecutiveFrames >;
    using ObjectConstPtr = std::shared_ptr< const ConsecutiveFrames >;

    static ObjectPtr create( const ProcFramePtr &frame1, const ProcFramePtr &frame2, const MapPtr &parent );

    ProcFramePtr frame1() const;
    ProcFramePtr frame2() const;

    void prepareFrame();
    void extract();
    void track();

protected:
    ConsecutiveFrames( const ProcFramePtr &frame1, const ProcFramePtr &frame2, const MapPtr &parent );

};

}
