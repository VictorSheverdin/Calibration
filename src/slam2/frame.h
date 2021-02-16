#pragma once

#include <vector>
#include <memory>

#include "src/common/supportclasses.h"

#include "src/common/colorpoint.h"
#include "src/common/image.h"
#include "src/common/projectionmatrix.h"

#include "parameters.h"

#include "alias.h"

namespace slam2 {

class StereoCameraMatrix;
class StereoDistorsionCoefficients;

class Frame : public std::enable_shared_from_this< Frame >, protected Parent_Weak_Ptr< StereoFrame >
{
    friend class FinalStereoFrame;

public:
    using ObjectClass = Frame;
    using ObjectPtr = std::shared_ptr< Frame >;
    using ObjectConstPtr = std::shared_ptr< const Frame >;

    virtual ~Frame() = default;

    void setTime( const std::chrono::time_point< std::chrono::system_clock > &time );
    const std::chrono::time_point< std::chrono::system_clock > &time() const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    std::shared_ptr< StereoFrame > parentStereoFrame() const;
    std::shared_ptr< Map > parentMap() const;
    std::shared_ptr< System > parentSystem() const;

    TrackPtr createTrack();

    const std::vector< TrackPtr > &tracks() const;

protected:
    Frame( const StereoFramePtr &parent );

    std::chrono::time_point< std::chrono::system_clock > _time;

    cv::Mat _cameraMatrix;

    std::vector< TrackPtr > _tracks;

    void setTracks( const std::vector< TrackPtr > &value );

};

class FinalFrame : public Frame
{
public:
    using ObjectClass = FinalFrame;
    using ParentClass = Frame;
    using ObjectPtr = std::shared_ptr< FinalFrame >;
    using ObjectConstPtr = std::shared_ptr< const FinalFrame >;

    static ObjectPtr create( const FinalStereoFramePtr &parent );

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    void addPoint( const FinalPointPtr &point );

protected:
    FinalFrame( const FinalStereoFramePtr &parent );

    std::vector< FinalPointPtr > _points;

private:
    void initialize();

};

class ProcFrame : public Frame
{
    friend class ProcStereoFrame;
    friend class FlowTracker;
    friend class CPUFlowTracker;
    friend class GPUFlowTracker;
    friend class FeatureTrackerAndMatcher;
    friend class SiftTracker;
    friend class OrbTracker;
    friend class AKazeTracker;
    friend class SuperGlueTracker;
public:
    using ObjectClass = ProcFrame;
    using ParentClass = Frame;
    using ObjectPtr = std::shared_ptr< ProcFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcFrame >;

    static ObjectPtr create( const ProcStereoFramePtr &parent );

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

    void setMask( const cv::Mat &mask );

    cv::Mat mask() const;

    FlowPointPtr createFlowPoint( const size_t index );
    FeaturePointPtr createFeaturePoint( const size_t index );

    FlowPointPtr flowPoint( const size_t index ) const;
    FeaturePointPtr featurePoint( const size_t index ) const;

    const std::map< size_t, FlowPointPtr > &flowPoints() const;
    std::vector< FlowPointPtr > flowPointsVector();

    const std::map< size_t, FeaturePointPtr > &featurePoints() const;
    std::vector< FeaturePointPtr > featurePointsVector();

    CvImage drawPoints() const;
    CvImage drawTracks() const;

    std::vector< ProcPointPtr > recoverPoints() const;
    size_t recoverPointsCount() const;

    cv::Scalar color( const cv::Point2f &point ) const;

protected:
    ProcFrame( const ProcStereoFramePtr &parent );

    StampedImage _image;

    cv::Mat _mask;

    std::vector< cv::Mat > _imagePyramid;

    cv::Mat _distCoefficients;

    std::vector< cv::Point2f > _cornerPoints;
    std::vector< cv::Point2f > _undistCornerPoints;

    std::map< size_t, FlowPointPtr > _flowPoints;

    std::vector< cv::KeyPoint > _keyPoints;
    std::vector< cv::KeyPoint > _undistKeyPoints;
    cv::Mat _descriptors;

    std::map< size_t, FeaturePointPtr > _featurePoints;

    static const cv::Scalar _recoverTrackColor;
    static const cv::Scalar _otherTrackColor;

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

class DoubleFrame : public std::enable_shared_from_this< DoubleFrame >, protected Parent_Weak_Ptr< Map >
{
public:
    using ObjectClass = DoubleFrame;
    using ObjectPtr = std::shared_ptr< DoubleFrame >;
    using ObjectConstPtr = std::shared_ptr< const DoubleFrame >;

    virtual ~DoubleFrame() = default;

    void set( const FramePtr &frame1, const FramePtr &frame2 );

    const FramePtr &frame1();
    const FramePtr &frame2();

    FrameConstPtr frame1() const;
    FrameConstPtr frame2() const;

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

    const FramePtr &leftFrame();
    const FramePtr &rightFrame();

    FrameConstPtr leftFrame() const;
    FrameConstPtr rightFrame() const;

    void setCameraMatrices( const StereoCameraMatrix &value );

    cv::Mat leftProjectionMatrix() const;
    cv::Mat rightProjectionMatrix() const;

    StereoProjectionMatrix projectionMatrix() const;

    void setRotation( const cv::Mat &value );
    const cv::Mat &rotation() const;

    void setTranslation( const cv::Mat &value );
    const cv::Mat &translation() const;

    void setRightRotation( const cv::Mat &value );
    const cv::Mat &rightRotation() const;

    void setRightTranslation( const cv::Mat &value );
    const cv::Mat &rightTranslation() const;

    std::vector< ColorPoint3d > sparseCloud() const;

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

    FinalFramePtr leftFrame();
    FinalFramePtr rightFrame();

    FinalFrameConstPtr leftFrame() const;
    FinalFrameConstPtr rightFrame() const;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    void replace( ProcStereoFramePtr source );

protected:
    FinalStereoFrame( const MapPtr &parent );

    std::vector< FinalStereoPointPtr > _points;

};

class ProcStereoFrame : public StereoFrame
{
    friend class CPUFlowTracker;
    friend class GPUFlowTracker;
    friend class SiftTracker;
    friend class OrbTracker;
    friend class AKazeTracker;
    friend class SuperGlueTracker;
public:
    using ObjectClass = ProcStereoFrame;
    using ParentClass = StereoFrame;
    using ObjectPtr = std::shared_ptr< ProcStereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const ProcStereoFrame >;

    static ObjectPtr create( const MapPtr &parent );

    void load( const StampedStereoImage &image );

    void setMask( const StereoMat &value );

    void extract();
    void match();

    size_t triangulatePoints();
    size_t triangulateTracks();

    double recoverPose();

    ProcFramePtr leftFrame();
    ProcFramePtr rightFrame();

    ProcFrameConstPtr leftFrame() const;
    ProcFrameConstPtr rightFrame() const;

    void setDistorsionCoefficients( const StereoDistorsionCoefficients &value );

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    FlowStereoPointPtr createFlowPoint( const size_t leftIndex, const size_t rightIndex );
    FeatureStereoPointPtr createFeaturePoint( const size_t leftIndex , const size_t rightIndex );

    CvImage drawPoints() const;
    CvImage drawTracks() const;
    CvImage drawStereo() const;

    const std::vector< ProcStereoPointPtr > &stereoPoints() const;

protected:
    ProcStereoFrame( const MapPtr &parent );

    void setImagePyramid( const std::vector< cv::Mat > &leftPyramid, const std::vector< cv::Mat > &rightPyramid );

    void setKeyPoints( const std::vector< cv::KeyPoint > &left, const std::vector< cv::KeyPoint > &right );
    void setDescriptors( const cv::Mat &left, const cv::Mat &right );

    std::vector< ProcStereoPointPtr > _points;

};

class ConsecutiveFrame : public DoubleFrame
{
public:
    using ObjectClass = ConsecutiveFrame;
    using ParentClass = DoubleFrame;
    using ObjectPtr = std::shared_ptr< ConsecutiveFrame >;
    using ObjectConstPtr = std::shared_ptr< const ConsecutiveFrame >;

    static ObjectPtr create( const ProcFramePtr &frame1, const ProcFramePtr &frame2, const MapPtr &parent );

    ProcFramePtr frame1();
    ProcFramePtr frame2();

    ProcFrameConstPtr frame1() const;
    ProcFrameConstPtr frame2() const;

    void extract();
    void track();

protected:
    ConsecutiveFrame( const ProcFramePtr &frame1, const ProcFramePtr &frame2, const MapPtr &parent );

};

class ConsecutiveStereoFrame
{
public:
    using ObjectClass = ConsecutiveStereoFrame;
    using ObjectPtr = std::shared_ptr< ConsecutiveStereoFrame >;
    using ObjectConstPtr = std::shared_ptr< const ConsecutiveStereoFrame >;

    ConsecutiveStereoFrame( const ProcStereoFramePtr &prevFrame, const ProcStereoFramePtr &nextFrame );

    void set( const ProcStereoFramePtr &prevFrame, const ProcStereoFramePtr &nextFrame );

    const ProcStereoFramePtr &prevFrame() const;
    const ProcStereoFramePtr &nextFrame() const;

protected:
    ProcStereoFramePtr _prevFrame;
    ProcStereoFramePtr _nextFrame;

};

}
