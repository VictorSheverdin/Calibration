#include "src/common/precompiled.h"

#include "framepoint.h"

#include "frame.h"

namespace slam2 {

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

// FlowPoint
FlowPoint::FlowPoint( const ProcFramePtr &parentFrame, const size_t index )
    : ParentClass( parentFrame, index )
{
}

FlowPoint::ObjectPtr FlowPoint::shared_from_this()
{
    return std::dynamic_pointer_cast< FlowPoint >( Point2::shared_from_this() );
}

FlowPoint::ObjectConstPtr FlowPoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FlowPoint >( Point2::shared_from_this() );
}

const cv::Point2f &FlowPoint::point() const
{
    return parentFrame()->cornerPoint( _index );
}

// FeaturePoint
FeaturePoint::FeaturePoint(const ProcFramePtr &parentFrame, const size_t index )
    : ParentClass( parentFrame, index )
{
}

FeaturePoint::ObjectPtr FeaturePoint::shared_from_this()
{
    return std::dynamic_pointer_cast< FeaturePoint >( Point2::shared_from_this() );
}

FeaturePoint::ObjectConstPtr FeaturePoint::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FeaturePoint >( Point2::shared_from_this() );
}

// StereoPoint
StereoPoint::StereoPoint( const Point2Ptr &leftPoint, const Point2Ptr &rightPoint )
{
    set( leftPoint, rightPoint );
}

void StereoPoint::set( const Point2Ptr &leftPoint, const Point2Ptr &rightPoint )
{
    _leftPoint = leftPoint;
    _rightPoint = rightPoint;
}

Point2Ptr StereoPoint::leftPoint() const
{
    return _leftPoint.lock();
}

Point2Ptr StereoPoint::rightPoint() const
{
    return _rightPoint.lock();
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

}
