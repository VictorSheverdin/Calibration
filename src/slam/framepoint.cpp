#include "src/common/precompiled.h"

#include "framepoint.h"

#include "frame.h"

#include "src/common/functions.h"

namespace slam {

    // PointBase
    PointBase::PointBase()
    {
    }

    // MonoPoint
    MonoPoint::MonoPoint()
    {
    }

    void MonoPoint::setStereoPoint( const AdjacentPtr point )
    {
        m_stereoPoint = point;
    }

    void MonoPoint::clearStereoPoint()
    {
        m_stereoPoint.reset();
    }

    MonoPoint::AdjacentPtr MonoPoint::stereoPoint() const
    {
        return m_stereoPoint.lock();
    }

    void MonoPoint::setNextPoint( const AdjacentPtr point )
    {
        m_nextPoint = point;
    }

    void MonoPoint::clearNextPoint()
    {
        m_nextPoint.reset();
    }

    MonoPoint::AdjacentPtr MonoPoint::nextPoint() const
    {
        return m_nextPoint.lock();
    }

    void MonoPoint::setPrevPoint( const AdjacentPtr point )
    {
        m_prevPoint = point;
    }

    void MonoPoint::clearPrevPoint()
    {
        m_prevPoint.reset();
    }

    MonoPoint::AdjacentPtr MonoPoint::prevPoint() const
    {
        return m_prevPoint.lock();
    }

    void MonoPoint::setWorldPoint( const WorldPointPtr point )
    {
        m_worldPoint = point;
    }

    void MonoPoint::clearWorldPoint()
    {
        m_worldPoint.reset();
    }

    MonoPoint::WorldPointPtr MonoPoint::worldPoint() const
    {
        return m_worldPoint.lock();
    }

    void MonoPoint::drawTrack( CvImage *target ) const
    {
        if ( prevPoint() ) {

            drawLine( target, prevPoint()->point(), point() );

            prevPoint()->drawTrack( target );


        }

    }

    // ProcessedPoint
    ProcessedPoint::ProcessedPoint( const FramePtr parentFrame , const size_t keyPointIndex )
        : m_parentFrame( parentFrame ), m_keyPointIndex( keyPointIndex )
    {
    }

    ProcessedPoint::PointPtr ProcessedPoint::create( const FramePtr parentFrame, const size_t keyPointIndex )
    {
        return PointPtr( new ProcessedPoint( parentFrame, keyPointIndex ) );
    }

    const cv::Point2f &ProcessedPoint::point() const
    {
        return m_parentFrame.lock()->m_keyPoints[ m_keyPointIndex ].pt;
    }

    const cv::Scalar &ProcessedPoint::color() const
    {
        return m_parentFrame.lock()->m_colors[ m_keyPointIndex ];
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

    DoublePoint::MonoPointPtr DoublePoint::monoFramePoint1() const
    {
        return m_point1;
    }

    DoublePoint::MonoPointPtr DoublePoint::monoFramePoint2() const
    {
        return m_point2;
    }

    // StereoPoint
    StereoPoint::StereoPoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint )
        : DoublePoint( leftPoint, rightPoint )
    {
    }

    StereoPoint::MonoPointPtr StereoPoint::leftFramePoint() const
    {
        return monoFramePoint1();
    }

    StereoPoint::MonoPointPtr StereoPoint::rightFramePoint() const
    {
        return monoFramePoint2();
    }

    cv::Point2f StereoPoint::leftPoint() const
    {
        return leftFramePoint()->point();
    }

    cv::Point2f StereoPoint::rightPoint() const
    {
        return rightFramePoint()->point();
    }

    // AdjacentPoint
    AdjacentPoint::AdjacentPoint( const MonoPointPtr previousPoint, const MonoPointPtr nextPoint )
        : DoublePoint( previousPoint, nextPoint )
    {
    }

    AdjacentPoint::MonoPointPtr AdjacentPoint::previousFramePoint() const
    {
        return monoFramePoint1();
    }

    AdjacentPoint::MonoPointPtr AdjacentPoint::nextFramePoint() const
    {
        return monoFramePoint2();
    }

    cv::Point2f AdjacentPoint::previousPoint() const
    {
        return previousFramePoint()->point();
    }

    cv::Point2f AdjacentPoint::nextPoint() const
    {
        return nextFramePoint()->point();
    }

}
