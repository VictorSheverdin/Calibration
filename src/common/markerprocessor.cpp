#include "precompiled.h"

#include "markerprocessor.h"

#include "functions.h"

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

// ArucoMarkerList
std::vector< cv::Point2f > ArucoMarkerList::points() const
{
    std::vector< cv::Point2f > ret;

    for ( auto i = begin(); i != end(); ++i )
        ret.insert( ret.end(), i->corners().begin(), i->corners().end() );

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

// ArucoProcessor
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

    m_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

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

void ArucoProcessor::setInterval( const double value )
{
    m_interval = value;
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

double ArucoProcessor::interval() const
{
    return m_interval;
}

bool ArucoProcessor::detectMarkers( const CvImage &image, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners )
{
    if ( markerIds && markerCorners ) {

        cv::aruco::detectMarkers( image, m_dictionary, *markerCorners, *markerIds, m_parameters );

        if ( markerIds->empty() || markerCorners->size() != markerIds->size() )
            return false;

        return true;

    }

    return false;
}

bool ArucoProcessor::processFrame( const Frame &frame, CvImage *view, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners )
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

bool ArucoProcessor::processFrame( const Frame &frame, CvImage *view, ArucoMarkerList *markers )
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

std::vector< cv::Point3f > ArucoProcessor::calcCorners( const ArucoMarkerList &list )
{
    std::vector< cv::Point3f > ret;

    const int firstId = 10;
    const int markersInRow = 5;

    static const double multiplier = m_size + m_interval;

    for ( auto &i : list ) {
        // Calculate offset index

        auto offsetId = i.id() - firstId;

        int row = offsetId / markersInRow;
        int col = offsetId % markersInRow;

        ret.push_back( cv::Point3f( row * multiplier, col * multiplier, 0 ) );
        ret.push_back( cv::Point3f( row * multiplier,  col * multiplier + m_size, 0 ) );
        ret.push_back( cv::Point3f( row * multiplier + m_size, col * multiplier + m_size, 0 ) );
        ret.push_back( cv::Point3f( row * multiplier + m_size, col * multiplier, 0 ) );

    }

    return ret;

}
