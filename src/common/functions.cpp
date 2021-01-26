#include "precompiled.h"

#include "functions.h"

#include <opencv2/core/eigen.hpp>

void drawTraceLines( CvImage &image, const unsigned int count )
{
    if ( !image.empty() && count > 0 ) {

        auto step = image.height() / count;

        for ( unsigned int i = 1; i < count; ++i )
            cv::line( image, cv::Point( 0, i * step ), cv::Point( image.width(), i * step ), cv::Scalar( 100, 255, 100, 100 ) );

    }

}

CvImage resizeTo( const CvImage &image, const unsigned int size )
{
    CvImage ret;

    if ( !image.empty() ) {
        auto maxSize = std::max( image.width(), image.height() );

        if ( maxSize != 0 ) {
            double scaleFactor = static_cast<double>( size ) / static_cast<double>( maxSize );

            cv::resize( image, ret, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LANCZOS4 );

        }

    }

    return ret;

}

CvImage stackImages(const CvImage &leftImage, const CvImage &rightImage, const double factor )
{
    auto normFactor = std::max( 0.0, std::min( 1.0, factor ) );

    CvImage result( std::max( leftImage.height(), rightImage.height() ),
                    leftImage.width() * normFactor + rightImage.width(),
                    leftImage.type(), cv::Scalar( 0, 0, 0, 0) );

    leftImage.copyTo( result( cv::Rect( 0, 0, leftImage.width(), leftImage.height() ) ) );
    rightImage.copyTo( result( cv::Rect( result.width() - rightImage.width(), 0, rightImage.width(), rightImage.height() ) ) );

    return result;

}

CvImage makeOverlappedPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage )
{
    return stackImages( leftPreviewImage, rightPreviewImage, 0.5 );
}

CvImage makeStraightPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage )
{
    return stackImages( leftPreviewImage, rightPreviewImage, 1 );
}

CvImage colorizeDisparity( const cv::Mat &disparity )
{
    double min, max;

    cv::minMaxIdx( disparity, &min, &max );
    double multiplier = 255.0 / (max - min);

    cv::Mat converted;

    disparity.convertTo( converted, CV_8U, multiplier, -min * multiplier );

    cv::Mat colored;

    cv::applyColorMap( converted, colored, cv::COLORMAP_JET );

    return colored;

}

bool drawFeaturePoint( CvImage *target, const cv::Point2f &pt, const int radius, const cv::Scalar &color )
{
    if ( !target )
        return false;

    cv::circle( *target, pt, radius, color, -1 );

    return true;
}

bool drawFeaturePoints( CvImage *target, const std::vector< cv::Point2f > &points, const int radius , const cv::Scalar &color )
{
    if ( !target )
        return false;

#pragma omp parallel for
    for ( size_t i = 0; i < points.size(); ++i )
        drawFeaturePoint( target, points[ i ], radius, color );

    return true;

}

bool drawFeaturePoints( CvImage *target, const std::vector< cv::KeyPoint > &keypoints, const int radius, const cv::Scalar &color )
{
    if ( !target )
        return false;

#pragma omp parallel for
    for ( size_t i = 0; i < keypoints.size(); ++i )
        drawFeaturePoint( target, keypoints[ i ].pt, radius, color );

    return true;

}

void drawLine( CvImage *target, const cv::Point2f &pt1, const cv::Point2f &pt2, const cv::Scalar &color )
{
    cv::line( *target, pt1, pt2, color, 1/*, cv::LINE_AA*/ );
}

void drawLabel( CvImage *target, const std::string text, int height, int fontFace, const int thickness, const cv::Scalar &color )
{
    if ( target ) {

        auto fontScale = cv::getFontScaleFromHeight( fontFace, height, thickness );

        auto fontSize = cv::getTextSize( text, fontFace, fontScale, thickness, nullptr );

        cv::putText( *target, text, cv::Point( target->width() - fontSize.width, target->height() - fontSize.height ),
                     fontFace, fontScale, color, thickness, cv::LINE_AA );

    }

}

Eigen::Quaterniond mat2Quaternion( const cv::Mat &R )
{
    Eigen::Matrix3d eigenMat;

    cv::cv2eigen( R, eigenMat );

    return Eigen::Quaterniond( eigenMat );
}

cv::Mat quaternion2Mat( const Eigen::Quaterniond &q )
{
    cv::Mat ret;

    cv::eigen2cv( q.toRotationMatrix(), ret );

    return ret;
}
