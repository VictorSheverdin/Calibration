#pragma once

#include "image.h"

#include "VimbaCPP/Include/VimbaCPP.h"

void drawTraceLines( CvImage &image, const unsigned int count );

CvImage resizeTo( const CvImage &image, unsigned int size );

CvImage stackImages( const CvImage &leftImage, const CvImage &rightImage, const double factor = 1.0 );

CvImage makeOverlappedPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );
CvImage makeStraightPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );

CvImage colorizeDisparity( const cv::Mat &disparity );

void checkVimbaStatus( VmbErrorType status, std::string message );

bool drawFeaturePoint( CvImage *target, const cv::Point2f &pt, const int radius = 3, const cv::Scalar &color = cv::Scalar( 0, 0, 255, 255 ) );
bool drawFeaturePoints( CvImage *target, const std::vector< cv::Point2f > &points, const int radius = 3, const cv::Scalar &color = cv::Scalar( 0, 0, 255, 255 ) );
bool drawFeaturePoints( CvImage *target, const std::vector< cv::KeyPoint > &keypoints, const int radius = 3, const cv::Scalar &color = cv::Scalar( 0, 0, 255, 255 ) );

void drawLine( CvImage *target, const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) );

void drawLabel( CvImage *target, const std::string text, int height, int fontFace = cv::FONT_HERSHEY_TRIPLEX, const int thickness = 1, const cv::Scalar &color = cv::Scalar( 255, 255, 255, 255 ) );

template <typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );

}

template <typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::VimbaSystem &system, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( system.GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );

}

template <typename FeatureT>
FeatureT getVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key )
{
    FeatureT ret;

    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->GetValue( ret ), "Could not get " + key );

    return ret;

}

void vimbaRunCommand( AVT::VmbAPI::VimbaSystem &system, const std::string &key );

void vimbaRunCommand(AVT::VmbAPI::CameraPtr camera, const std::string &key );

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

