#include "src/common/precompiled.h"

#include "framepoint.h"

#include "frame.h"

namespace slam {

    // PointBase
    PointBase::PointBase()
    {
    }

    // MonoPoint
    MonoPoint::MonoPoint()
    {
    }

    void MonoPoint::setTrack( const TrackPtr track )
    {
        m_track = track;
    }

    MonoPoint::TrackPtr MonoPoint::track() const
    {
        return m_track;
    }

    // FeaturePoint
    FeaturePoint::FeaturePoint( const FramePtr parentFrame , const size_t keyPointIndex )
        : m_parentFrame( parentFrame ), m_keyPointIndex( keyPointIndex )
    {
    }

    FeaturePoint::PointPtr FeaturePoint::create( const FramePtr parentFrame, const size_t keyPointIndex )
    {
        return PointPtr( new FeaturePoint( parentFrame, keyPointIndex ) );
    }

    const cv::Point2f &FeaturePoint::point() const
    {
        return m_parentFrame.lock()->m_keyPoints[ m_keyPointIndex ].pt;
    }

    // DoublePoint
    DoublePoint::DoublePoint( const MonoPointPtr point1, const MonoPointPtr point2 )
    {
        setMonoPoints( point1, point2 );
    }

    void DoublePoint::setMonoPoints( const MonoPointPtr point1, const MonoPointPtr point2 )
    {
        m_point1 = point1;
        m_point2 = point2;
    }

    DoublePoint::MonoPointPtr DoublePoint::monoPoint1() const
    {
        return m_point1;
    }

    DoublePoint::MonoPointPtr DoublePoint::monoPoint2() const
    {
        return m_point2;
    }

    // StereoPoint
    StereoPoint::StereoPoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint )
        : DoublePoint( leftPoint, rightPoint )
    {
    }

    const cv::Point2f &StereoPoint::leftPoint() const
    {
        return leftMonoPoint()->point();
    }

    const cv::Point2f &StereoPoint::rightPoint() const
    {
        return rightMonoPoint()->point();
    }

    StereoPoint::MonoPointPtr StereoPoint::leftMonoPoint() const
    {
        return monoPoint1();
    }

    StereoPoint::MonoPointPtr StereoPoint::rightMonoPoint() const
    {
        return monoPoint2();
    }

    void StereoPoint::setWorldPoint( const WorldPointPtr value )
    {
        m_worldPoint = value;
    }

    StereoPoint::WorldPointPtr StereoPoint::worldPoint() const
    {
        return m_worldPoint;
    }

    // ConsecutivePoint
    ConsecutivePoint::ConsecutivePoint( const MonoPointPtr point1, const MonoPointPtr point2 )
        : DoublePoint( point1, point2 )
    {
    }

}
