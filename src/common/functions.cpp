#include "precompiled.h"

#include "functions.h"

void drawTraceLines( CvImage &image, const unsigned int count )
{
    if ( !image.empty() && count > 0 ) {

        auto step = image.height() / count;

        for ( unsigned int i = 1; i < count; ++i )
            cv::line( image, cv::Point( 0, i * step ), cv::Point( image.width(), i * step ), cv::Scalar( 100, 255, 100, 100 ) );

    }

}

CvImage resizeTo( const CvImage &image, unsigned int size )
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

void checkVimbaStatus( VmbErrorType status, std::string message )
{
    if (status != VmbErrorSuccess)
    {
        throw std::runtime_error(
            message + "; status = " + std::to_string(status) );
    }
}

bool drawFeaturePoint( CvImage *target, const cv::Point2f &pt, const int radius, const cv::Scalar &color )
{
    if ( !target )
        return false;

    cv::circle( *target, pt, radius, color, -1 );

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
    cv::line( *target, pt1, pt2, color );
}

void vimbaRunCommand( AVT::VmbAPI::VimbaSystem &system, const std::string &key )
{
    AVT::VmbAPI::FeaturePtr feature;
    checkVimbaStatus( system.GetFeatureByName( key.data(), feature ),
        "Could not access " + key );

    checkVimbaStatus( feature->RunCommand(), "Could run command " + key );

    bool bIsCommandDone = false;

    do
    {
        if( VmbErrorSuccess != SP_ACCESS( feature )->IsCommandDone( bIsCommandDone ) )
        {
            break;
        }
    } while( false == bIsCommandDone );

}

void vimbaRunCommand( AVT::VmbAPI::CameraPtr camera, const std::string &key )
{
    AVT::VmbAPI::FeaturePtr feature;
    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key );

    checkVimbaStatus( feature->RunCommand(), "Could run command " + key );

    bool bIsCommandDone = false;

    do
    {
        if( VmbErrorSuccess != SP_ACCESS( feature )->IsCommandDone( bIsCommandDone ) )
        {
            break;
        }
    } while( false == bIsCommandDone );

}

