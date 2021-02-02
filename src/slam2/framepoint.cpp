#include "src/common/precompiled.h"

#include "framepoint.h"

#include "frame.h"

namespace slam2 {

// Point2
void Point2::setParentTrack( const TrackPtr &track )
{
    _parentTrack = track;
}

TrackPtr Point2::parentTrack() const
{
    return _parentTrack.lock();
}

void Point2::setTrackIndex( const size_t index )
{
    _trackIndex = index;
}

size_t Point2::trackIndex() const
{
    return _trackIndex;
}

// FramePoint
FramePoint::ObjectPtr FramePoint::shared_from_this()
{
    return std::dynamic_pointer_cast< FramePoint >( Point2::shared_from_this() );
}

FramePoint::ObjectConstPtr FramePoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FramePoint >( Point2::shared_from_this() );
}

// ProcPoint
ProcPoint::ProcPoint(const ProcFramePtr &parentFrame , const size_t index )
{
    setParentFrame( parentFrame );
    setIndex( index );
}

void ProcPoint::setParentFrame( const ProcFramePtr &parent )
{
    setParentPointer( parent );
}

ProcFramePtr ProcPoint::parentFrame() const
{
    return parentPointer();
}

void ProcPoint::setIndex( const size_t index )
{
    _index = index;
}

size_t ProcPoint::index() const
{
    return _index;
}

ProcPoint::ObjectPtr ProcPoint::shared_from_this()
{
    return std::dynamic_pointer_cast< ProcPoint >( Point2::shared_from_this() );
}

ProcPoint::ObjectConstPtr ProcPoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const ProcPoint >( Point2::shared_from_this() );
}

cv::Scalar ProcPoint::color() const
{
    return parentFrame()->image().at< cv::Vec3b >( point2d() );
}

// FlowPoint
FlowPoint::FlowPoint( const ProcFramePtr &parentFrame, const size_t index )
    : ParentClass( parentFrame, index )
{
}

FlowPoint::ObjectPtr FlowPoint::create( const ProcFramePtr &parentFrame, const size_t index )
{
    return ObjectPtr( new FlowPoint( parentFrame, index ) );
}

FlowPoint::ObjectPtr FlowPoint::shared_from_this()
{
    return std::dynamic_pointer_cast< FlowPoint >( Point2::shared_from_this() );
}

FlowPoint::ObjectConstPtr FlowPoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FlowPoint >( Point2::shared_from_this() );
}

const cv::Point2f &FlowPoint::undistortedPoint() const
{
    return parentFrame()->undistortedCornerPoint( _index );
}

const cv::Point2f &FlowPoint::point2d() const
{
    return parentFrame()->cornerPoint( _index );
}

// FeaturePoint
FeaturePoint::FeaturePoint( const ProcFramePtr &parentFrame, const size_t index )
    : ParentClass( parentFrame, index )
{
}

FeaturePoint::ObjectPtr FeaturePoint::create( const ProcFramePtr &parentFrame, const size_t index )
{
    return ObjectPtr( new FeaturePoint( parentFrame, index ) );
}

FeaturePoint::ObjectPtr FeaturePoint::shared_from_this()
{
    return std::dynamic_pointer_cast< FeaturePoint >( Point2::shared_from_this() );
}

FeaturePoint::ObjectConstPtr FeaturePoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FeaturePoint >( Point2::shared_from_this() );
}

const cv::Point2f &FeaturePoint::undistortedPoint() const
{
    return parentFrame()->undistortedKeyPoint( _index ).pt;
}

const cv::Point2f &FeaturePoint::point2d() const
{
    return parentFrame()->keyPoint( _index ).pt;
}

// DoublePoint
DoublePoint::DoublePoint( const Point2Ptr &point1, const Point2Ptr &point2 )
{
    set( point1, point2 );
}

void DoublePoint::set( const Point2Ptr &point1, const Point2Ptr &point2 )
{
    _point1 = point1;
    _point2 = point2;
}

Point2Ptr DoublePoint::point1() const
{
    return _point1.lock();
}

Point2Ptr DoublePoint::point2() const
{
    return _point2.lock();
}

// StereoPoint
StereoPoint::StereoPoint( const Point2Ptr &leftPoint, const Point2Ptr &rightPoint )
    : DoublePoint( leftPoint, rightPoint )
{
}

Point2Ptr StereoPoint::leftPoint() const
{
    return point1();
}

Point2Ptr StereoPoint::rightPoint() const
{
    return point2();
}

bool StereoPoint::isPoint3dExist()
{
    return _point3d.has_value();
}

void StereoPoint::setPoint3d( const cv::Point3f &value )
{
    _point3d = value;
}

const cv::Point3f &StereoPoint::point3d() const
{
    return _point3d.value();
}

cv::Scalar StereoPoint::color() const
{
    return .5 * ( leftPoint()->color() + rightPoint()->color() );
}

// ProcStereoPoint
ProcStereoPoint::ProcStereoPoint( const ProcPointPtr &leftPoint, const ProcPointPtr &rightPoint )
    : StereoPoint( leftPoint, rightPoint )
{
}

ProcPointPtr ProcStereoPoint::leftPoint() const
{
    return std::dynamic_pointer_cast< ProcPoint >( StereoPoint::leftPoint() );
}

ProcPointPtr ProcStereoPoint::rightPoint() const
{
    return std::dynamic_pointer_cast< ProcPoint >( StereoPoint::rightPoint() );
}

// FlowStereoPoint
FlowStereoPoint::FlowStereoPoint( const FlowPointPtr &leftPoint, const FlowPointPtr &rightPoint )
    : ProcStereoPoint( leftPoint, rightPoint )
{
}

FlowPointPtr FlowStereoPoint::leftPoint() const
{
    return std::dynamic_pointer_cast< FlowPoint >( StereoPoint::leftPoint() );
}

FlowPointPtr FlowStereoPoint::rightPoint() const
{
    return std::dynamic_pointer_cast< FlowPoint >( StereoPoint::rightPoint() );
}

FlowStereoPoint::ObjectPtr FlowStereoPoint::create( const FlowPointPtr &leftPoint, const FlowPointPtr &rightPoint )
{
    return ObjectPtr( new FlowStereoPoint( leftPoint, rightPoint ) );
}

// FeatureStereoPoint
FeatureStereoPoint::FeatureStereoPoint( const FeaturePointPtr &leftPoint, const FeaturePointPtr &rightPoint )
    : ProcStereoPoint( leftPoint, rightPoint )
{
}

FeaturePointPtr FeatureStereoPoint::leftPoint() const
{
    return std::dynamic_pointer_cast< FeaturePoint >( StereoPoint::leftPoint() );
}

FeaturePointPtr FeatureStereoPoint::rightPoint() const
{
    return std::dynamic_pointer_cast< FeaturePoint >( StereoPoint::rightPoint() );
}

FeatureStereoPoint::ObjectPtr FeatureStereoPoint::create( const FeaturePointPtr &leftPoint, const FeaturePointPtr &rightPoint )
{
    return ObjectPtr( new FeatureStereoPoint( leftPoint, rightPoint ) );
}

// ConsecutivePoints
ConsecutivePoints::ConsecutivePoints( const Point2Ptr &point1, const Point2Ptr &point2 )
    : DoublePoint( point1, point2 )
{
}


}
