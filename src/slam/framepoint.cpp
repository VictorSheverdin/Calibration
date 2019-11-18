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

    MonoPoint::AdjacentPtr MonoPoint::stereoPoint() const
    {
        return m_stereoPoint.lock();
    }

    void MonoPoint::setNextPoint( const AdjacentPtr point )
    {
        m_nextPoint = point;
    }

    MonoPoint::AdjacentPtr MonoPoint::nextPoint() const
    {
        return m_nextPoint.lock();
    }

    void MonoPoint::setPrevPoint( const AdjacentPtr point )
    {
        m_prevPoint = point;
    }

    MonoPoint::AdjacentPtr MonoPoint::prevPoint() const
    {
        return m_prevPoint.lock();
    }

    void MonoPoint::setWorldPoint( const WorldPointPtr point )
    {
        m_worldPoint = point;
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

    DoublePoint::MonoPointPtr DoublePoint::monoFramePoint1() const
    {
        return m_point1;
    }

    DoublePoint::MonoPointPtr DoublePoint::monoFramePoint2() const
    {
        return m_point2;
    }

    // SpatialPoint
    SpatialPoint::SpatialPoint()
    {
    }

    void SpatialPoint::setSpatialPoint( const cv::Vec3d &value )
    {
        m_spatialPoint = value;
    }

    const cv::Vec3d &SpatialPoint::spatialPoint() const
    {
        return m_spatialPoint;
    }

    void SpatialPoint::setSpatialColor( const cv::Vec4b &value )
    {
        m_spatialColor = value;
    }

    const cv::Vec4b &SpatialPoint::spatialColor() const
    {
        return m_spatialColor;
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

    // SpatialStereoPoint
    SpatialStereoPoint::SpatialStereoPoint( const StereoPoint &stereoPoint )
        : StereoPoint( stereoPoint )
    {
    }

    SpatialStereoPoint::SpatialStereoPoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint )
        : StereoPoint( leftPoint, rightPoint )
    {
    }


}
