#include "src/common/precompiled.h"

#include "framepoint.h"

#include "frame.h"

namespace slam2 {

// Point2
Point2::Point2( const FramePtr &parentFrame )
    : Parent_Weak_Ptr< Frame >( parentFrame )
{
}

void Point2::setParentFrame( const FramePtr &parent )
{
    setParentPointer( parent );
}

void Point2::setParentTrack( const TrackPtr &track )
{
    _parentTrack = track;
}

FramePtr Point2::parentFrame() const
{
    return parentPointer();
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

void Point2::setStereoPoint( const StereoPointPtr &value )
{
    _stereoPoint = value;
}

void Point2::clearStereoPoint()
{
    _stereoPoint.reset();
}

StereoPointPtr Point2::stereoPoint() const
{
    return _stereoPoint.lock();
}

// FinalPoint
FinalPoint::FinalPoint( const FinalFramePtr &parentFrame )
    : Point2( parentFrame )
{
}

FinalPoint::ObjectPtr FinalPoint::create( const FinalFramePtr &parentFrame )
{
    return ObjectPtr( new FinalPoint( parentFrame ) );
}

FinalPoint::ObjectPtr FinalPoint::shared_from_this()
{
    return std::dynamic_pointer_cast< FinalPoint >( Point2::shared_from_this() );
}

FinalPoint::ObjectConstPtr FinalPoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinalPoint >( Point2::shared_from_this() );
}

const cv::Point2f &FinalPoint::undistortedPoint() const
{
    return ColorPoint2d::point();
}

const cv::Point2f &FinalPoint::point2d() const
{
    return ColorPoint2d::point();
}

cv::Scalar FinalPoint::color() const
{
    return ColorPoint2d::color();
}

// ProcPoint
ProcPoint::ProcPoint(const ProcFramePtr &parentFrame , const size_t index )
    : Point2( parentFrame )
{
    setIndex( index );
}

void ProcPoint::setParentFrame( const ProcFramePtr &parent )
{
    setParentPointer( parent );
}

ProcFramePtr ProcPoint::parentFrame() const
{
    return std::dynamic_pointer_cast< ProcFrame >( Point2::parentFrame() );
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
    return parentFrame()->color( point2d() );
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
DoublePoint::DoublePoint()
{
}

void DoublePoint::set( const Point2Ptr &point1, const Point2Ptr &point2 )
{
    _point1 = point1;
    _point2 = point2;
}

const Point2Ptr &DoublePoint::point1()
{
    return _point1;
}

const Point2Ptr &DoublePoint::point2()
{
    return _point2;
}

Point2ConstPtr DoublePoint::point1() const
{
    return _point1;
}

Point2ConstPtr DoublePoint::point2() const
{
    return _point2;
}

// StereoPoint
StereoPoint::StereoPoint()
{
}

void StereoPoint::set( const Point2Ptr &point1, const Point2Ptr &point2 )
{
    DoublePoint::set( point1, point2 );

    if ( point1 )
        point1->setStereoPoint( shared_from_this() );

    if ( point2 )
        point2->setStereoPoint( shared_from_this() );
}

const Point2Ptr &StereoPoint::leftPoint()
{
    return point1();
}

const Point2Ptr &StereoPoint::rightPoint()
{
    return point2();
}

Point2ConstPtr StereoPoint::leftPoint() const
{
    return point1();
}

Point2ConstPtr StereoPoint::rightPoint() const
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

StereoPoint::ObjectPtr StereoPoint::shared_from_this()
{
    return std::dynamic_pointer_cast< StereoPoint >( DoublePoint::shared_from_this() );
}

StereoPoint::ObjectConstPtr StereoPoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const StereoPoint >( DoublePoint::shared_from_this() );
}

// FinalStereoPoint
FinalStereoPoint::FinalStereoPoint()
{
}

FinalPointPtr FinalStereoPoint::leftPoint()
{
    return std::dynamic_pointer_cast< FinalPoint >( StereoPoint::leftPoint() );
}

FinalPointPtr FinalStereoPoint::rightPoint()
{
    return std::dynamic_pointer_cast< FinalPoint >( StereoPoint::rightPoint() );
}

FinalPointConstPtr FinalStereoPoint::leftPoint() const
{
    return std::dynamic_pointer_cast< const FinalPoint >( StereoPoint::leftPoint() );
}

FinalPointConstPtr FinalStereoPoint::rightPoint() const
{
    return std::dynamic_pointer_cast< const FinalPoint >( StereoPoint::rightPoint() );
}

FinalStereoPoint::ObjectPtr FinalStereoPoint::create()
{
    return ObjectPtr( new FinalStereoPoint() );
}

// ProcStereoPoint
ProcStereoPoint::ProcStereoPoint()
{
}

ProcPointPtr ProcStereoPoint::leftPoint()
{
    return std::dynamic_pointer_cast< ProcPoint >( StereoPoint::leftPoint() );
}

ProcPointPtr ProcStereoPoint::rightPoint()
{
    return std::dynamic_pointer_cast< ProcPoint >( StereoPoint::rightPoint() );
}

ProcPointConstPtr ProcStereoPoint::leftPoint() const
{
    return std::dynamic_pointer_cast< const ProcPoint >( StereoPoint::leftPoint() );
}

ProcPointConstPtr ProcStereoPoint::rightPoint() const
{
    return std::dynamic_pointer_cast< const ProcPoint >( StereoPoint::rightPoint() );
}

// FlowStereoPoint
FlowStereoPoint::FlowStereoPoint()
{
}

FlowPointPtr FlowStereoPoint::leftPoint()
{
    return std::dynamic_pointer_cast< FlowPoint >( StereoPoint::leftPoint() );
}

FlowPointPtr FlowStereoPoint::rightPoint()
{
    return std::dynamic_pointer_cast< FlowPoint >( StereoPoint::rightPoint() );
}

FlowPointConstPtr FlowStereoPoint::leftPoint() const
{
    return std::dynamic_pointer_cast< const FlowPoint >( StereoPoint::leftPoint() );
}

FlowPointConstPtr FlowStereoPoint::rightPoint() const
{
    return std::dynamic_pointer_cast< const FlowPoint >( StereoPoint::rightPoint() );
}

FlowStereoPoint::ObjectPtr FlowStereoPoint::create()
{
    return ObjectPtr( new FlowStereoPoint() );
}

// FeatureStereoPoint
FeatureStereoPoint::FeatureStereoPoint()
{
}

FeaturePointPtr FeatureStereoPoint::leftPoint()
{
    return std::dynamic_pointer_cast< FeaturePoint >( StereoPoint::leftPoint() );
}

FeaturePointPtr FeatureStereoPoint::rightPoint()
{
    return std::dynamic_pointer_cast< FeaturePoint >( StereoPoint::rightPoint() );
}

FeaturePointConstPtr FeatureStereoPoint::leftPoint() const
{
    return std::dynamic_pointer_cast< const FeaturePoint >( StereoPoint::leftPoint() );
}

FeaturePointConstPtr FeatureStereoPoint::rightPoint() const
{
    return std::dynamic_pointer_cast< const FeaturePoint >( StereoPoint::rightPoint() );
}

FeatureStereoPoint::ObjectPtr FeatureStereoPoint::create()
{
    return ObjectPtr( new FeatureStereoPoint() );
}

// ConsecutivePoints
ConsecutivePoints::ConsecutivePoints()
{
}


}
