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

using ArucoMarkerList = std::list< ArucoMarker >;

class ArucoProcessor
{
public:
    ArucoProcessor();

    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );

    bool resizeFlag() const;
    unsigned int frameMaximumFlag() const;

    ArucoMarkerList detectMarkers( const CvImage &image ) ;

    bool processFrame( const Frame &frame, CvImage *view, ArucoMarkerList *markers );
    bool processPreview( const Frame &frame, CvImage *preview );

protected:
    cv::Ptr< cv::aruco::Dictionary > m_dictionary ;

    bool m_resizeFlag;
    int m_frameMaximumSize;

private:
    void initialize();

} ;
