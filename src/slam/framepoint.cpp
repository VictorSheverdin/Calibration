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
    MonoPoint::MonoPoint( const FramePtr &parentFrame )
        : m_parentFrame( parentFrame )
    {
        initialize();
    }

    void MonoPoint::initialize()
    {
    }

    MonoPoint::FramePtr MonoPoint::parentFrame() const
    {
        return m_parentFrame.lock();
    }

    void MonoPoint::setStereoPoint( const AdjacentPtr &point )
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

    void MonoPoint::setNextPoint( const AdjacentPtr &point )
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

    void MonoPoint::setPrevPoint( const AdjacentPtr &point )
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

    void MonoPoint::setMapPoint( const MapPointPtr &point )
    {
        m_mapPoint = point;

       if ( point )
            point->addFramePoint( shared_from_this() );

    }

    void MonoPoint::clearMapPoint()
    {
        auto mapPoint = this->mapPoint();

        if ( mapPoint )
            mapPoint->removeFramePoint( shared_from_this() );

        m_mapPoint.reset();
    }

    MonoPoint::MapPointPtr MonoPoint::mapPoint() const
    {
        return m_mapPoint.lock();
    }

    size_t MonoPoint::connectedPointsCount() const
    {
        size_t ret = 0;

        if ( stereoPoint() )
            ++ret;

        for ( auto i = prevPoint(); i; i = i->prevPoint() ) {
            ++ret;

            if ( i->stereoPoint() )
                ++ret;

        }

        for ( auto i = nextPoint(); i; i = i->nextPoint() ) {
            ++ret;

            if ( i->stereoPoint() )
                ++ret;

        }

        return ret;
    }

    size_t MonoPoint::prevTrackLenght() const
    {
        size_t ret = 1;

        for ( auto i = prevPoint(); i; i = i->prevPoint() )
            ++ret;

        return ret;

    }

    size_t MonoPoint::nextTrackLenght() const
    {
        size_t ret = 1;

        for ( auto i = nextPoint(); i; i = i->nextPoint() )
            ++ret;

        return ret;

    }

    Eigen::Matrix< double, 2, 1 > MonoPoint::eigenPoint() const
    {
        Eigen::Matrix< double, 2, 1 > ret;

        auto point = this->point();

        ret << point.x, point.y;

        return ret;
    }

    Eigen::Matrix< double, 3, 1 > MonoPoint::eigenStereoPoint() const
    {
        Eigen::Matrix< double, 3, 1 > ret;

        auto point = this->point();

        auto stereoPoint = this->stereoPoint();

        double stereoX = point.x;

        if ( stereoPoint ) {

            stereoX = stereoPoint->point().x;

        }

        ret << point.x, point.y, stereoX;

        return ret;

    }

    void MonoPoint::drawTrack( CvImage *target, const cv::Scalar &color ) const
    {
        drawPrevTrack( target, color );
        drawNextTrack( target, color );

    }

    void MonoPoint::drawPrevTrack( CvImage *target , const cv::Scalar &color ) const
    {
        auto prevPoint = this->prevPoint();

        if ( prevPoint ) {

            drawLine( target, prevPoint->point(), point(), color );

            prevPoint->drawPrevTrack( target, color );

        }

    }

    void MonoPoint::drawNextTrack( CvImage *target , const cv::Scalar &color ) const
    {
        auto nextPoint = this->nextPoint();

        if ( nextPoint ) {

            drawLine( target, point(), nextPoint->point(), color );

            nextPoint->drawNextTrack( target, color );

        }

    }

    // ProcessedPointBase
    ProcessedPointBase::ProcessedPointBase( const FramePtr &parentFrame )
        : MonoPoint( parentFrame )
    {
    }

    // FlowPoint
    FlowPoint::FlowPoint( const FramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
        : ProcessedPointBase( parentFrame ), m_point( point ), m_color( color )
    {
    }

    const cv::Point2f &FlowPoint::point() const
    {
        return m_point;
    }

    const cv::Scalar &FlowPoint::color() const
    {
        return m_color;
    }

    FlowPoint::ObjectPtr FlowPoint::create( const FramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
    {
        return ObjectPtr( new FlowPoint( parentFrame, point, color ) );
    }

    FlowPoint::FramePtr FlowPoint::parentFrame() const
    {
        return std::dynamic_pointer_cast< FlowFrame >( m_parentFrame.lock() );
    }

    // FeaturePoint
    FeaturePoint::FeaturePoint( const FramePtr &parentFrame , const size_t keyPointIndex )
        : ProcessedPointBase( parentFrame ), m_keyPointIndex( keyPointIndex )
    {
    }

    FeaturePoint::ObjectPtr FeaturePoint::create( const FramePtr &parentFrame, const size_t keyPointIndex )
    {
        return ObjectPtr( new FeaturePoint( parentFrame, keyPointIndex ) );
    }

    FeaturePoint::FramePtr FeaturePoint::parentFrame() const
    {
        return std::dynamic_pointer_cast< FeatureFrame >( m_parentFrame.lock() );
    }

    const cv::Point2f &FeaturePoint::point() const
    {
        return parentFrame()->m_keyPoints[ m_keyPointIndex ].pt;
    }

    const cv::Scalar &FeaturePoint::color() const
    {
        return parentFrame()->m_colors[ m_keyPointIndex ];
    }

    const cv::KeyPoint &FeaturePoint::keyPoint() const
    {
        return parentFrame()->m_keyPoints[ m_keyPointIndex ];
    }

    cv::Mat FeaturePoint::descriptor() const
    {
        if ( parentFrame()->m_descriptors.rows > static_cast< int >( m_keyPointIndex ) )
            return parentFrame()->m_descriptors.row( m_keyPointIndex );
        else
            return cv::Mat();
    }

    // FramePoint
    FramePoint::FramePoint( const FramePtr &parentFrame )
        : MonoPoint( parentFrame)
    {
    }

    FramePoint::FramePoint( const FramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
        : MonoPoint( parentFrame), ColorPoint2d( point, color )
    {
    }

    const cv::Point2f &FramePoint::point() const
    {
        return ColorPoint2d::point();
    }

    const cv::Scalar &FramePoint::color() const
    {
        return ColorPoint2d::color();
    }

    FramePoint::ObjectPtr FramePoint::create( const FramePtr &parentFrame )
    {
        return ObjectPtr( new FramePoint( parentFrame ) );
    }

    FramePoint::ObjectPtr FramePoint::create( const FramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
    {
        return ObjectPtr( new FramePoint( parentFrame, point, color ) );
    }

    FramePoint::FramePtr FramePoint::parentFrame() const
    {
        return std::dynamic_pointer_cast< Frame >( m_parentFrame.lock() );
    }

    void FramePoint::replace( const FeaturePointPtr &point )
    {
        if ( point ) {

            auto stereoPoint = point->stereoPoint();

            setStereoPoint( stereoPoint );

            if ( stereoPoint )
                stereoPoint->setStereoPoint( shared_from_this() );

            auto nextPoint = point->nextPoint();

            setNextPoint( nextPoint );

            if ( nextPoint )
                nextPoint->setPrevPoint( shared_from_this() );

            auto prevPoint = point->prevPoint();

            setPrevPoint( prevPoint );

            if ( prevPoint )
                prevPoint->setNextPoint( shared_from_this() );

            setMapPoint( point->mapPoint() );

            point->clearMapPoint();
            point->clearNextPoint();
            point->clearPrevPoint();
            point->clearStereoPoint();

        }

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

