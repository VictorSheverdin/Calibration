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

    for ( auto i = begin(); i != end(); ++i ) {
        bool foundFlag = false;
        for ( auto j = other.begin(); j != other.end(); ++j )
            if ( j->id() == i->id() ) {
                foundFlag = true;
                break;
            }
        if ( !foundFlag )
            return false;

    }

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

        cv::aruco::detectMarkers( image, m_dictionary, *markerCorners, *markerIds );

        if ( markerIds->empty() || markerCorners->size() != markerIds->size() )
            return false;

        return true;

    }

    return false;
}

bool ArucoProcessor::processFrame( const Frame &frame, CvImage *view, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners )
{
    if ( !frame.empty() ) {

        auto ret = detectMarkers( frame, markerIds, markerCorners );

        if (view) {
            frame.copyTo( *view );

            if ( ret && view )
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
            markers->push_back( ArucoMarker( markerIds[ i ], markerCorners[ i ] ) );

    }

    return ret;
}

bool ArucoProcessor::processPreview( const Frame &frame, CvImage *preview )
{
    bool ret = false;

    if ( !frame.empty() ) {

        auto extent = std::max( frame.width(), frame.height() );

        cv::Mat sourceFrame;

        if ( m_resizeFlag && extent > m_frameMaximumSize )
            sourceFrame = resizeTo( frame, m_frameMaximumSize );
        else
            frame.copyTo( sourceFrame );

        ret = processFrame( sourceFrame, preview, nullptr );

    }

    return ret;
}

std::vector< cv::Point3f > ArucoProcessor::calcCorners( const ArucoMarkerList &list )
{
    std::vector< cv::Point3f > ret;

    return ret;
}
