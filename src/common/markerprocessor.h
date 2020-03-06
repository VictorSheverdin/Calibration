#pragma once

#include "image.h"

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

    bool operator<( const ArucoMarker &other ) const;

    cv::Point2f centroid() const;

protected:
    int m_id ;
    std::vector< cv::Point2f > m_corners ;
} ;

class ArucoMarkerList : public std::set< ArucoMarker >
{
public:
    ArucoMarkerList() = default;

    std::vector< cv::Point2f > points() const;
    std::vector< cv::Point2f > centerPoints() const;

    bool operator==( const ArucoMarkerList& other ) const;

protected:

};

void fit( ArucoMarkerList *list1, ArucoMarkerList *list2 );

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

    bool processFrame( const CvImage &frame, CvImage *view, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners ) const;
    bool processFrame( const CvImage &frame, CvImage *view = nullptr, ArucoMarkerList *markers = nullptr ) const;

    std::vector< cv::Point3f > calcCorners( const ArucoMarkerList &list ) const;
    std::vector< cv::Point3f > calcCentroids( const ArucoMarkerList &list ) const;

protected:
    cv::Ptr< cv::aruco::Dictionary > m_dictionary ;
    cv::Ptr< cv::aruco::DetectorParameters > m_parameters ;

    static const int m_firstId = 10;
    static const int m_markersInRow = 5;

    bool m_resizeFlag;
    int m_frameMaximumSize;

    double m_size;
    double m_interval;

    bool detectMarkers( const CvImage &image, std::vector< int > *markerIds, std::vector< std::vector< cv::Point2f > > *markerCorners ) const ;

private:
    void initialize();

} ;
