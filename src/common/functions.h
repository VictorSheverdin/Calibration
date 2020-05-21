#pragma once

#include "image.h"

#include <tuple>

void drawTraceLines( CvImage &image, const unsigned int count );

CvImage resizeTo( const CvImage &image, const unsigned int size );
CvImage scale( const CvImage &image, const double factor );

CvImage stackImages( const CvImage &leftImage, const CvImage &rightImage, const double factor = 1.0 );

CvImage makeOverlappedPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );
CvImage makeStraightPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );

CvImage colorizeDisparity( const cv::Mat &disparity );

bool drawFeaturePoint( CvImage *target, const cv::Point2f &pt, const int radius = 3, const cv::Scalar &color = cv::Scalar( 0, 0, 255, 255 ) );
bool drawFeaturePoints( CvImage *target, const std::vector< cv::Point2f > &points, const int radius = 3, const cv::Scalar &color = cv::Scalar( 0, 0, 255, 255 ) );
bool drawFeaturePoints( CvImage *target, const std::vector< cv::KeyPoint > &keypoints, const int radius = 3, const cv::Scalar &color = cv::Scalar( 0, 0, 255, 255 ) );

void drawLine( CvImage *target, const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) );

void drawLabel( CvImage *target, const std::string text, int height, int fontFace = cv::FONT_HERSHEY_TRIPLEX, const int thickness = 1, const cv::Scalar &color = cv::Scalar( 255, 255, 255, 255 ) );

std::tuple< double, double, double, double > mat2Quaternion( const cv::Mat &R );

template < class T >
cv::Mat point3fToMat( const cv::Point3f &point )
{
    return cv::Mat_< T >( 3, 1 ) << point.x, point.y, point.z;
}

template < class T >
cv::Point3f matToPoint3f( const cv::Mat &mat )
{
    return cv::Point3f( mat.at< T >( 0, 0 ), mat.at< T >( 1, 0 ), mat.at< T >( 2, 0 ) );
}

