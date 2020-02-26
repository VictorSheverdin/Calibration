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

class ArucoMarkerList : public std::list< ArucoMarker >
{
public:
    ArucoMarkerList() = default;

    std::vector< cv::Point2f > points() const;

    bool operator==( const ArucoMarkerList& other ) const;

protected:

};

class ArucoProcessor
{
public:
    ArucoProcessor();

    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );
    void setSize( const double value );
    void setInterval( const double value );

    bool resizeFlag() const;
    unsigned int frameMaximumFlag() const;
    double size() const;
    double interval() const;

    bool processFrame( const Frame &frame, CvImage *view, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners );
    bool processFrame( const Frame &frame, CvImage *view, ArucoMarkerList *markers );
    bool processPreview( const Frame &frame, CvImage *preview );

    std::vector< cv::Point3f > calcCorners( const ArucoMarkerList &list );

protected:
    cv::Ptr< cv::aruco::Dictionary > m_dictionary ;

    bool m_resizeFlag;
    int m_frameMaximumSize;

    double m_size;
    double m_interval;

    bool detectMarkers( const CvImage &image, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners ) ;

private:
    void initialize();

} ;
