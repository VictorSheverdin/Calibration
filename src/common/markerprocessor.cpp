#include "precompiled.h"

#include "markerprocessor.h"

#include "functions.h"

#include "defs.h"

// ArucoMarker
ArucoMarker::ArucoMarker()
{
}

ArucoMarker::ArucoMarker( const int id, const std::vector< cv::Point2f > &corners )
{
    set( id, corners ) ;
}

void ArucoMarker::setId( const int value )
{
    m_id = value;
}

int ArucoMarker::id() const
{
    return m_id;
}

void ArucoMarker::setCorners( const std::vector< cv::Point2f > &value )
{
    m_corners = value;
}

const std::vector< cv::Point2f > &ArucoMarker::corners() const
{
    return m_corners;
}

void ArucoMarker::set( const int id, const std::vector< cv::Point2f > &corners )
{
    setId( id );
    setCorners( corners );
}

bool ArucoMarker::operator<( const ArucoMarker &other ) const
{
    return m_id < other.m_id;
}

cv::Point2f ArucoMarker::centroid() const
{
    cv::Moments mu = cv::moments( m_corners );

    if ( std::abs( mu.m00 ) > DOUBLE_EPS )
        return cv::Point2f( mu.m10 / mu.m00 , mu.m01 / mu.m00 );
    else
        return cv::Point2f();

}

// ArucoMarkerList
std::vector< cv::Point2f > ArucoMarkerList::points() const
{
    std::vector< cv::Point2f > ret;

    for ( auto i = begin(); i != end(); ++i )
        ret.insert( ret.end(), i->corners().begin(), i->corners().end() );

    return ret;

}

std::vector< cv::Point2f > ArucoMarkerList::centerPoints() const
{
    std::vector< cv::Point2f > ret;

    for ( auto i = begin(); i != end(); ++i )
        ret.push_back( i->centroid() );

    return ret;

}

bool ArucoMarkerList::operator==( const ArucoMarkerList& other ) const
{
    if ( size() != other.size() )
        return false;

    for ( auto i = begin(); i != end(); ++i )
        if ( other.find( *i ) == other.end() )
            return false;

    return true;

}

void fit( ArucoMarkerList *list1, ArucoMarkerList *list2 )
{
    if ( list1 && list2 ) {

        for ( auto i = list1->begin(); i != list1->end(); ) {
            if ( list2->find( *i ) == list2->end() )
                i = list1->erase( i );
            else
                ++i;

        }

        for ( auto i = list2->begin(); i != list2->end(); ) {
            if ( list1->find( *i ) == list1->end() )
                i = list2->erase( i );
            else
                ++i;

        }

    }
}

// ArucoProcessor
const double ArucoProcessor::m_intervalMultiplier = 34./177.;

ArucoProcessor::ArucoProcessor()
{
    initialize();
}

void ArucoProcessor::initialize()
{
    m_resizeFlag = false;
    m_frameMaximumSize = 500;

    m_dictionary = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_6X6_50 ) ;
    m_parameters = cv::aruco::DetectorParameters::create() ;

    m_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;

}

void ArucoProcessor::setResizeFlag( const bool value )
{
    m_resizeFlag = value;
}

void ArucoProcessor::setFrameMaximumSize( const unsigned int value )
{
    m_frameMaximumSize = value;
}

void ArucoProcessor::setSize( const double value )
{
    m_size = value;
}

bool ArucoProcessor::resizeFlag() const
{
    return m_resizeFlag;
}

unsigned int ArucoProcessor::frameMaximumFlag() const
{
    return m_frameMaximumSize;
}

double ArucoProcessor::size() const
{
    return m_size;
}

bool ArucoProcessor::detectMarkers( const CvImage &image, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners ) const
{
    if ( markerIds && markerCorners ) {

        cv::aruco::detectMarkers( image, m_dictionary, *markerCorners, *markerIds, m_parameters );

        if ( markerIds->empty() || markerCorners->size() != markerIds->size() )
            return false;

        return true;

    }

    return false;
}

bool ArucoProcessor::processFrame( const CvImage &frame, CvImage *view, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners ) const
{
    if ( !frame.empty() ) {

        cv::Mat sourceFrame;

        auto extent = std::max( frame.width(), frame.height() );

        if ( m_resizeFlag && extent > m_frameMaximumSize )
            sourceFrame = resizeTo( frame, m_frameMaximumSize );
        else
            frame.copyTo( sourceFrame );

        auto ret = detectMarkers( sourceFrame, markerIds, markerCorners );

        if ( view ) {
            *view = sourceFrame;

            if ( ret )
                cv::aruco::drawDetectedMarkers( *view, *markerCorners, *markerIds );

        }

        return ret;

    }

    return false;

}

bool ArucoProcessor::processFrame( const CvImage &frame, CvImage *view, ArucoMarkerList *markers ) const
{
    std::vector< int > markerIds;
    std::vector< std::vector< cv::Point2f > > markerCorners;

    auto ret = processFrame( frame, view, &markerIds, &markerCorners );

    if ( ret && markers ) {
        markers->clear();

        for ( size_t i = 0; i < markerIds.size(); ++i )
            markers->insert( ArucoMarker( markerIds[ i ], markerCorners[ i ] ) );


    }

    return ret;

}

std::vector< cv::Point3f > ArucoProcessor::calcCorners( const ArucoMarkerList &list ) const
{
    std::vector< cv::Point3f > ret;

    static const double multiplier = m_size * ( 1.0 + m_intervalMultiplier );

    for ( auto &i : list ) {
        // Calculate offset index

        auto offsetId = i.id() - m_firstId;

        int row = offsetId / m_markersInRow;
        int col = offsetId % m_markersInRow;

        ret.push_back( cv::Point3f( row * multiplier, col * multiplier, 0 ) );
        ret.push_back( cv::Point3f( row * multiplier,  col * multiplier + m_size, 0 ) );
        ret.push_back( cv::Point3f( row * multiplier + m_size, col * multiplier + m_size, 0 ) );
        ret.push_back( cv::Point3f( row * multiplier + m_size, col * multiplier, 0 ) );

    }

    return ret;

}

std::vector< cv::Point3f > ArucoProcessor::calcCentroids( const ArucoMarkerList &list ) const
{
    std::vector< cv::Point3f > ret;

    static const double multiplier = m_size * ( 1.0 + m_intervalMultiplier );

    static const double halfSize = m_size / 2.0;

    for ( auto &i : list ) {
        // Calculate offset index

        auto offsetId = i.id() - m_firstId;

        int row = offsetId / m_markersInRow;
        int col = offsetId % m_markersInRow;

        ret.push_back( cv::Point3f( row * multiplier + halfSize, col * multiplier + halfSize, 0 ) );

    }

    return ret;

}
