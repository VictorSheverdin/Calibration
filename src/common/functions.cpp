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
    cv::line( *target, pt1, pt2, color );
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

std::tuple< double, double, double, double > mat2Quaternion( const cv::Mat &R )
{
    double Q[ 4 ];

    double trace = R.at< double >( 0, 0 ) + R.at< double >( 1, 1 ) + R.at< double >( 2, 2 );

    if ( trace > 0.0 )
    {
        double s = sqrt( trace + 1.0 );
        Q[ 3 ] = ( s * 0.5 );
        s = 0.5 / s;
        Q[0] = ( ( R.at< double >( 2, 1 ) - R.at< double >( 1, 2 ) ) * s );
        Q[1] = ( ( R.at< double >( 0, 2 ) - R.at< double >( 2, 0 ) ) * s);
        Q[2] = ( ( R.at< double >( 1, 0 ) - R.at< double >( 0, 1 ) ) * s);
    }
    else
    {
        int i = R.at< double >( 0, 0 ) < R.at< double >( 1, 1 ) ? ( R.at< double >( 1, 1 ) < R.at< double >( 2, 2 ) ? 2 : 1 ) : ( R.at< double >( 0, 0 ) < R.at< double >( 2, 2 ) ? 2 : 0 );
        int j = ( i + 1 ) % 3;
        int k = ( i + 2 ) % 3;

        double s = sqrt( R.at< double >( i, i ) - R.at< double >( j, j ) - R.at< double >( k, k ) + 1.0 );
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = ( R.at< double >( k, j ) - R.at< double >( j, k ) ) * s;
        Q[j] = ( R.at< double >( j, i ) + R.at< double >( i, j ) ) * s;
        Q[k] = ( R.at< double >( k, i ) + R.at< double >( i, k ) ) * s;
    }

    return std::make_tuple( Q[ 0 ], Q[ 1 ], Q[ 2 ], Q[ 3 ] );

}
