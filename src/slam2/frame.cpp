#include "src/common/precompiled.h"

#include "frame.h"

#include "system.h"
#include "map.h"

#include "parameters.h"

#include "tracker.h"

namespace slam2 {

// Frame
Frame::Frame( const StereoFramePtr &parent )
    : Parent_Shared_Ptr< StereoFrame >( parent )
{
}

std::shared_ptr< StereoFrame > Frame::parentStereoFrame() const
{
    return parentPointer();
}

std::shared_ptr< Map > Frame::parentMap() const
{
    return parentStereoFrame()->parentMap();
}

std::shared_ptr< System > Frame::parentSystem() const
{
    return parentMap()->parentSystem();
}

// FinalFrame
FinalFrame::FinalFrame( const StereoFramePtr &parent )
    : Frame( parent )
{
}

FinalFrame::ObjectPtr FinalFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinalFrame >( Frame::shared_from_this() );
}

FinalFrame::ObjectConstPtr FinalFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinalFrame >( Frame::shared_from_this() );
}

void FinalFrame::setTime( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    _time = time;
}

const std::chrono::time_point< std::chrono::system_clock > &FinalFrame::time() const
{
    return _time;
}

void FinalFrame::setCameraMatrix( const cv::Mat &value )
{
    _cameraMatrix = value;
}

const cv::Mat &FinalFrame::cameraMatrix() const
{
    return _cameraMatrix;
}

// ProcFrame
ProcFrame::ProcFrame( const StereoFramePtr &parent )
    : FinalFrame( parent )
{
}

ProcFrame::ObjectPtr ProcFrame::create( const StereoFramePtr &parent )
{
    return ObjectPtr( new ProcFrame( parent ) );
}

void ProcFrame::load( const StampedImage &image )
{
    setTime( image.time() );

    _image = image;
}

void ProcFrame::buildPyramid()
{
    auto system = parentSystem();

    auto tracker = system->flowTracker();

    _imagePyramid = tracker->buildPyramid( _image );
}

void ProcFrame::extractCorners()
{
    auto system = parentSystem();

    auto tracker = system->flowTracker();

    size_t extractionCount = std::max( 0, static_cast< int >( system->parameters().cornerExtractionCount() )
                                       - static_cast< int >( _cornerPoints.size() ) );

    std::vector< cv::Point2f > cornerPoints;

    tracker->extractCorners( _image, cornersMask(), extractionCount, &cornerPoints );

    addCornerPoints( cornerPoints );

}

const cv::Point2f &ProcFrame::cornerPoint( const size_t index ) const
{
    return _cornerPoints[ index ];
}

ProcFrame::ObjectPtr ProcFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< ProcFrame >( Frame::shared_from_this() );
}

ProcFrame::ObjectConstPtr ProcFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const ProcFrame >( Frame::shared_from_this() );
}

void ProcFrame::setDistorsionCoefficients( const cv::Mat &value )
{
    _distCoefficients = value;
}

const cv::Mat &ProcFrame::distorsionCoefficients() const
{
    return _distCoefficients;
}

cv::Mat ProcFrame::cornersMask() const
{
    cv::Mat ret( _image.rows, _image.cols, CV_8U, cv::Scalar( 1 ) );

    auto system = parentSystem();

    auto tracker = system->flowTracker();

    auto distance = tracker->extractionDistance();

    for ( auto &i : _cornerPoints )
        cv::rectangle( ret, cv::Rect( i.x - distance, i.y - distance, distance * 2, distance * 2 ), cv::Scalar( 0 ) );

    return ret;
}

void ProcFrame::addCornerPoint( const cv::Point2f &point )
{
    _cornerPoints.push_back( point );
}

void ProcFrame::addCornerPoints( const std::vector< cv::Point2f > &points )
{
    _cornerPoints.insert( _cornerPoints.end(), points.begin(), points.end() );
}

const std::vector< cv::Mat > &ProcFrame::imagePyramid() const
{
    return _imagePyramid;
}

const std::vector< cv::Point2f > &ProcFrame::cornerPoints() const
{
    return _cornerPoints;
}

// StereoFrame
StereoFrame::StereoFrame( const MapPtr &parent )
    : Parent_Shared_Ptr< Map >( parent )
{
}

void StereoFrame::setLeftFrame( const FramePtr value )
{
    _leftFrame = value;
}

void StereoFrame::setRightFrame( const FramePtr value )
{
    _rightFrame = value;
}

std::shared_ptr< Map > StereoFrame::parentMap() const
{
    return parentPointer();
}

std::shared_ptr< System > StereoFrame::parentSystem() const
{
    return parentMap()->parentSystem();
}

// FinalStereoFrame
FinalStereoFrame::FinalStereoFrame( const MapPtr &parent )
    : StereoFrame( parent )
{
}

FinalStereoFrame::ObjectPtr FinalStereoFrame::create( const MapPtr &parent )
{
    return ObjectPtr( new FinalStereoFrame( parent ) );
}

FinalFramePtr FinalStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FinalFrame >( _leftFrame );
}

FinalFramePtr FinalStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FinalFrame >( _rightFrame );
}

void FinalStereoFrame::setCameraMatrices( const StereoCameraMatrix &value )
{
    leftFrame()->setCameraMatrix( value.left() );
    rightFrame()->setCameraMatrix( value.right() );
}

FinalStereoFrame::ObjectPtr FinalStereoFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinalStereoFrame >( StereoFrame::shared_from_this() );
}

FinalStereoFrame::ObjectConstPtr FinalStereoFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinalStereoFrame >( StereoFrame::shared_from_this() );
}

// ProcStereoFrame
ProcStereoFrame::ProcStereoFrame( const MapPtr &parent )
    : FinalStereoFrame( parent )
{
}

ProcStereoFrame::ObjectPtr ProcStereoFrame::create( const MapPtr &parent )
{
    return ObjectPtr( new ProcStereoFrame( parent ) );
}

void ProcStereoFrame::load( const StampedStereoImage &image )
{
    setLeftFrame( ProcFrame::create( shared_from_this() ) );
    setRightFrame( ProcFrame::create( shared_from_this() ) );

    leftFrame()->load( image.leftImage() );
    rightFrame()->load( image.rightImage() );
}

void ProcStereoFrame::buildPyramid()
{
    leftFrame()->buildPyramid();
    rightFrame()->buildPyramid();
}

void ProcStereoFrame::matchCorners()
{
    auto system = parentSystem();

    auto tracker = system->flowTracker();

    std::vector< FlowTrackResult > matchResults;

    tracker->match( leftFrame()->imagePyramid(), rightFrame()->imagePyramid(), leftFrame()->cornerPoints(), &matchResults );

}

ProcFramePtr ProcStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< ProcFrame >( _leftFrame );
}

ProcFramePtr ProcStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcFrame >( _rightFrame );
}

void ProcStereoFrame::setDistorsionCoefficients( const StereoDistorsionCoefficients &value )
{
    leftFrame()->setDistorsionCoefficients( value.left() );
    rightFrame()->setDistorsionCoefficients( value.right() );
}

ProcStereoFrame::ObjectPtr ProcStereoFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< ProcStereoFrame >( StereoFrame::shared_from_this() );
}

ProcStereoFrame::ObjectConstPtr ProcStereoFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const ProcStereoFrame >( StereoFrame::shared_from_this() );
}

}
