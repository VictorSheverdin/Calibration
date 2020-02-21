#include "precompiled.h"

#include "markerprocessor.h"

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

// ArucoProcessor
ArucoProcessor::ArucoProcessor()
{
    initialize();
}

void ArucoProcessor::initialize()
{
    m_dictionary = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_6X6_50 ) ;
}

void ArucoProcessor::detectMarkers( const CvImage &image )
{
    std::vector< int > markerIds;
    std::vector< std::vector< cv::Point2f > > markerCorners;

    cv::aruco::detectMarkers( image, m_dictionary, markerCorners, markerIds );
    // cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);

}
