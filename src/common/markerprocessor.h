#pragma once

#include "src/common/image.h"

#include <opencv2/aruco.hpp>

class ArucoMarker
{
public:
    ArucoMarker();
    ArucoMarker( const int id, const std::vector< cv::Point2f > &corners );

    void setId( const int value );
    int id() const ;

    void setCorners( const std::vector< cv::Point2f > &value ) ;
    const std::vector< cv::Point2f > &corners() const ;

    void set( const int id, const std::vector< cv::Point2f > &corners );

protected:
    int m_id ;
    std::vector< cv::Point2f > m_corners ;
} ;

class ArucoProcessor
{
public:
    ArucoProcessor();

    void detectMarkers( const CvImage &image ) ;

protected:
    cv::Ptr< cv::aruco::Dictionary > m_dictionary ;

private:
    void initialize();

} ;
