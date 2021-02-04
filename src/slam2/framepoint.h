#pragma once

#include <memory>

#include "src/common/colorpoint.h"

#include "src/common/supportclasses.h"

#include "alias.h"

namespace slam2 {

class Track;

class Point2 : public std::enable_shared_from_this< Point2 >
{
    friend class Track;
    friend class StereoPoint;

public:
    using ObjectClass = Point2;
    using ObjectPtr = std::shared_ptr< Point2 >;
    using ObjectConstPtr = std::shared_ptr< const Point2 >;

    virtual ~Point2() = default;

    virtual const cv::Point2f &point2d() const = 0;
    virtual cv::Scalar color() const = 0;

    TrackPtr parentTrack() const;
    size_t trackIndex() const;

    StereoPointPtr stereoPoint() const;

protected:
    Point2() = default;

    TrackWeak _parentTrack;
    size_t _trackIndex;

    StereoPointWeak _stereoPoint;

    void setParentTrack( const TrackPtr &track );
    void setTrackIndex( const size_t index );

    void setStereoPoint( const StereoPointPtr &value );

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

    virtual const cv::Point2f &undistortedPoint() const = 0;
    cv::Scalar color() const override;

protected:
    ProcPoint( const ProcFramePtr &parentFrame, const size_t index );

    void setParentFrame( const ProcFramePtr &parent );

    void setIndex( const size_t index );

    size_t _index;
};

class FlowPoint : public ProcPoint
{
    friend class ProcFrame;

public:
    using ObjectClass = FlowPoint;
    using ParentClass = ProcPoint;
    using ObjectPtr = std::shared_ptr< FlowPoint >;
    using ObjectConstPtr = std::shared_ptr< const FlowPoint >;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    const cv::Point2f &undistortedPoint() const override;
    const cv::Point2f &point2d() const override;

protected:
    FlowPoint( const ProcFramePtr &parentFrame, const size_t index );

    static ObjectPtr create( const ProcFramePtr &parentFrame, const size_t index );
};

class FeaturePoint : public ProcPoint
{
    friend class ProcFrame;

public:
    using ObjectClass = FeaturePoint;
    using ParentClass = ProcPoint;
    using ObjectPtr = std::shared_ptr< FeaturePoint >;
    using ObjectConstPtr = std::shared_ptr< const FeaturePoint >;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

    const cv::Point2f &undistortedPoint() const override;
    const cv::Point2f &point2d() const override;

protected:
    FeaturePoint( const ProcFramePtr &parentFrame, const size_t index );

    static ObjectPtr create( const ProcFramePtr &parentFrame, const size_t index );
};

class DoublePoint : public std::enable_shared_from_this< DoublePoint >
{
public:
    using ObjectClass = DoublePoint;
    using ObjectPtr = std::shared_ptr< DoublePoint >;
    using ObjectConstPtr = std::shared_ptr< const DoublePoint >;

    virtual ~DoublePoint() = default;

    void set( const Point2Ptr &point1, const Point2Ptr &point2 );

    Point2Ptr point1() const;
    Point2Ptr point2() const;

protected:
    DoublePoint();

    Point2Weak _point1;
    Point2Weak _point2;
};

class StereoPoint : public DoublePoint
{
public:
    using ObjectClass = StereoPoint;
    using ParentClass = DoublePoint;
    using ObjectPtr = std::shared_ptr< StereoPoint >;
    using ObjectConstPtr = std::shared_ptr< const StereoPoint >;

    void set( const Point2Ptr &point1, const Point2Ptr &point2 );

    Point2Ptr leftPoint() const;
    Point2Ptr rightPoint() const;

    bool isPoint3dExist();

    void setPoint3d( const cv::Point3f &value );
    const cv::Point3f &point3d() const;

    cv::Scalar color() const;

    ObjectPtr shared_from_this();
    ObjectConstPtr shared_from_this() const;

protected:
    StereoPoint();

    std::optional< cv::Point3f > _point3d;

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
    ProcStereoPoint();
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

    static ObjectPtr create();

protected:
    FlowStereoPoint();
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

    static ObjectPtr create();

protected:
    FeatureStereoPoint();
};

class ConsecutivePoints : public DoublePoint
{
public:
    using ObjectClass = ConsecutivePoints;
    using ParentClass = DoublePoint;
    using ObjectPtr = std::shared_ptr< ConsecutivePoints >;
    using ObjectConstPtr = std::shared_ptr< const ConsecutivePoints >;

protected:
    ConsecutivePoints();
};

}
