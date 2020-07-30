#include "src/common/precompiled.h"

#include "framepoint.h"

#include "frame.h"

#include "src/common/functions.h"

namespace slam {

    // MonoPoint
    MonoPoint::MonoPoint( const MonoFramePtr &parentFrame )
    {
        initialize();

        setParentFrame( parentFrame );
    }

    void MonoPoint::initialize()
    {
        m_error = 0.;
    }

    void MonoPoint::setParentFrame( const MonoFramePtr &parent )
    {
        m_parentFrame = parent;
    }

    MonoFramePtr MonoPoint::parentFrame() const
    {
        return m_parentFrame.lock();
    }

    void MonoPoint::setStereoPoint( const MonoPointPtr &point )
    {
        m_stereoPoint = point;
    }

    void MonoPoint::clearStereoPoint()
    {
        m_stereoPoint.reset();
    }

    MonoPointPtr MonoPoint::stereoPoint() const
    {
        return m_stereoPoint.lock();
    }

    void MonoPoint::setNextPoint( const MonoPointPtr &point )
    {
        m_nextPoint = point;
    }

    void MonoPoint::clearNextPoint()
    {
        m_nextPoint.reset();
    }

    MonoPointPtr MonoPoint::nextPoint() const
    {
        return m_nextPoint.lock();
    }

    void MonoPoint::setPrevPoint( const MonoPointPtr &point )
    {
        m_prevPoint = point;
    }

    void MonoPoint::clearPrevPoint()
    {
        m_prevPoint.reset();
    }

    MonoPointPtr MonoPoint::prevPoint() const
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

    MapPointPtr MonoPoint::mapPoint() const
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

    double MonoPoint::error() const
    {
        return m_error;
    }

    void MonoPoint::setError( const double value )
    {
        m_error = value;
    }

    void MonoPoint::replace( const MonoPointPtr &point )
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

    void MonoPoint::dissolve()
    {
        auto stereoPoint = this->stereoPoint();

        if ( stereoPoint )
            stereoPoint->clearStereoPoint();

        clearStereoPoint();

        auto prevPoint = this->prevPoint();
        auto nextPoint = this->nextPoint();

        if ( prevPoint )
            prevPoint->setNextPoint( nextPoint );

        if ( nextPoint )
            nextPoint->setPrevPoint( prevPoint );

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
    ProcessedPointBase::ProcessedPointBase( const MonoFramePtr &parentFrame )
        : MonoPoint( parentFrame )
    {
    }

    // FlowPoint
    FlowPoint::FlowPoint( const FlowFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
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

    FlowPoint::ObjectPtr FlowPoint::create( const FlowFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
    {
        return ObjectPtr( new FlowPoint( parentFrame, point, color ) );
    }

    FlowFramePtr FlowPoint::parentFrame() const
    {
        return std::dynamic_pointer_cast< FlowFrame >( m_parentFrame.lock() );
    }

    double FlowPoint::misstake() const
    {
        return m_misstake;
    }

    void FlowPoint::setMisstake( const double value )
    {
        m_misstake = value;
    }

    // FeaturePoint
    FeaturePoint::FeaturePoint( const FeatureFramePtr &parentFrame , const size_t keyPointIndex )
        : ProcessedPointBase( parentFrame ), m_keyPointIndex( keyPointIndex )
    {
    }

    FeaturePoint::ObjectPtr FeaturePoint::create( const FeatureFramePtr &parentFrame, const size_t keyPointIndex )
    {
        return ObjectPtr( new FeaturePoint( parentFrame, keyPointIndex ) );
    }

    FeatureFramePtr FeaturePoint::parentFrame() const
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

    // FinishedFramePoint
    FinishedFramePoint::FinishedFramePoint( const FinishedFramePtr &parentFrame )
        : MonoPoint( parentFrame)
    {
    }

    FinishedFramePoint::FinishedFramePoint( const FinishedFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
        : MonoPoint( parentFrame), ColorPoint2d( point, color )
    {
    }

    const cv::Point2f &FinishedFramePoint::point() const
    {
        return ColorPoint2d::point();
    }

    const cv::Scalar &FinishedFramePoint::color() const
    {
        return ColorPoint2d::color();
    }

    FinishedFramePoint::ObjectPtr FinishedFramePoint::create( const FinishedFramePtr &parentFrame )
    {
        return ObjectPtr( new FinishedFramePoint( parentFrame ) );
    }

    FinishedFramePoint::ObjectPtr FinishedFramePoint::create( const FinishedFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color )
    {
        return ObjectPtr( new FinishedFramePoint( parentFrame, point, color ) );
    }

    FinishedFramePtr FinishedFramePoint::parentFrame() const
    {
        return std::dynamic_pointer_cast< FinishedFrame >( m_parentFrame.lock() );
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

    MonoPointPtr DoublePoint::monoFramePoint1() const
    {
        return m_point1;
    }

    MonoPointPtr DoublePoint::monoFramePoint2() const
    {
        return m_point2;
    }

    cv::Point2f DoublePoint::point1() const
    {
        return monoFramePoint1()->point();
    }

    cv::Point2f DoublePoint::point2() const
    {
        return monoFramePoint2()->point();
    }

    double DoublePoint::distance() const
    {
        return cv::norm( point2() - point1() );
    }

    // StereoPoint
    StereoPoint::StereoPoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint )
        : DoublePoint( leftPoint, rightPoint )
    {
    }

    MonoPointPtr StereoPoint::leftFramePoint() const
    {
        return monoFramePoint1();
    }

    MonoPointPtr StereoPoint::rightFramePoint() const
    {
        return monoFramePoint2();
    }

    cv::Point2f StereoPoint::leftPoint() const
    {
        return point1();
    }

    cv::Point2f StereoPoint::rightPoint() const
    {
        return point2();
    }

    // ConsecutivePoint
    ConsecutivePoint::ConsecutivePoint( const MonoPointPtr previousPoint, const MonoPointPtr nextPoint )
        : DoublePoint( previousPoint, nextPoint )
    {
    }

    MonoPointPtr ConsecutivePoint::startFramePoint() const
    {
        return monoFramePoint1();
    }

    MonoPointPtr ConsecutivePoint::endFramePoint() const
    {
        return monoFramePoint2();
    }

    cv::Point2f ConsecutivePoint::startPoint() const
    {
        return point1();
    }

    cv::Point2f ConsecutivePoint::endPoint() const
    {
        return point2();
    }

}

