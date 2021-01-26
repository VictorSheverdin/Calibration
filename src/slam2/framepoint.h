#pragma once

#include <memory>

#include "src/common/colorpoint.h"

#include "src/common/supportclasses.h"

#include "alias.h"

namespace slam2 {

class Point2 : public std::enable_shared_from_this< Point2 >
{
public:
    using ObjectClass = Point2;
    using ObjectPtr = std::shared_ptr< Point2 >;
    using ObjectConstPtr = std::shared_ptr< const Point2 >;

    virtual ~Point2() = default;

protected:
    Point2() = default;

};

class FramePoint : public Point2, public ColorPoint2d
{
public:
    using ObjectClass = FramePoint;
    using ParentClass = Point2;
    using ObjectPtr = std::shared_ptr< FramePoint >;
    using ObjectConstPtr = std::shared_ptr< const FramePoint >;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

protected:
    FramePoint() = default;
};

class ProcPoint : public Point2, protected Parent_Shared_Ptr< ProcFrame >
{
public:
    using ObjectClass = ProcPoint;
    using ParentClass = Point2;
    using ObjectPtr = std::shared_ptr< ProcPoint >;
    using ObjectConstPtr = std::shared_ptr< const ProcPoint >;

    ProcFramePtr parentFrame() const;

    size_t index() const;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

protected:
    ProcPoint( const ProcFramePtr &parentFrame, const size_t index );

    void setParentFrame( const ProcFramePtr &parent );

    void setIndex( const size_t index );

    size_t _index;
};

class FlowPoint : public ProcPoint
{
public:
    using ObjectClass = FlowPoint;
    using ParentClass = ProcPoint;
    using ObjectPtr = std::shared_ptr< FlowPoint >;
    using ObjectConstPtr = std::shared_ptr< const FlowPoint >;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    const cv::Point2f &point() const;

protected:
    FlowPoint( const ProcFramePtr &parentFrame, const size_t index );
};

class FeaturePoint : public ProcPoint
{
public:
    using ObjectClass = FeaturePoint;
    using ParentClass = ProcPoint;
    using ObjectPtr = std::shared_ptr< FeaturePoint >;
    using ObjectConstPtr = std::shared_ptr< const FeaturePoint >;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

protected:
    FeaturePoint( const ProcFramePtr &parentFrame, const size_t index );
};

class StereoPoint : public std::enable_shared_from_this< StereoPoint >
{
public:
    using ObjectClass = StereoPoint;
    using ObjectPtr = std::shared_ptr< StereoPoint >;
    using ObjectConstPtr = std::shared_ptr< const StereoPoint >;

    virtual ~StereoPoint() = default;

    void set( const Point2Ptr &leftPoint, const Point2Ptr &rightPoint );

    Point2Ptr leftPoint() const;
    Point2Ptr rightPoint() const;

protected:
    StereoPoint( const Point2Ptr &leftPoint, const Point2Ptr &rightPoint );

    Point2Weak _leftPoint;
    Point2Weak _rightPoint;

};

class ProcStereoPoint : public StereoPoint
{
public:
    using ObjectClass = ProcStereoPoint;
    using ParentClass = StereoPoint;
    using ObjectPtr = std::shared_ptr< ProcStereoPoint >;
    using ObjectConstPtr = std::shared_ptr< const ProcStereoPoint >;

    ProcPointPtr leftPoint() const;
    ProcPointPtr rightPoint() const;

protected:
    ProcStereoPoint( const ProcPointPtr &leftPoint, const ProcPointPtr &rightPoint );
};

class FlowStereoPoint : public ProcStereoPoint
{
public:
    using ObjectClass = FlowStereoPoint;
    using ParentClass = ProcStereoPoint;
    using ObjectPtr = std::shared_ptr< FlowStereoPoint >;
    using ObjectConstPtr = std::shared_ptr< const FlowStereoPoint >;

    FlowPointPtr leftPoint() const;
    FlowPointPtr rightPoint() const;

protected:
    FlowStereoPoint( const FlowPointPtr &leftPoint, const FlowPointPtr &rightPoint );
};

class FeatureStereoPoint : public ProcStereoPoint
{
public:
    using ObjectClass = FeatureStereoPoint;
    using ParentClass = ProcStereoPoint;
    using ObjectPtr = std::shared_ptr< FeatureStereoPoint >;
    using ObjectConstPtr = std::shared_ptr< const FeatureStereoPoint >;

    FeaturePointPtr leftPoint() const;
    FeaturePointPtr rightPoint() const;

protected:
    FeatureStereoPoint( const FeaturePointPtr &leftPoint, const FeaturePointPtr &rightPoint );
};

}
