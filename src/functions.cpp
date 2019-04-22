#include "precompiled.h"

#include "functions.h"

void drawTraceLines( CvImage &image, const unsigned int count )
{
    if ( !image.empty() && count > 0 ) {

        auto step = image.height() / count;

        for ( auto i = 1; i < count; ++i )
            cv::line( image, cv::Point( 0, i * step ), cv::Point( image.width(), i * step ), cv::Scalar( 100, 255, 100, 100 ) );

    }

}

CvImage resizeTo( const CvImage &image, unsigned int size )
{
    CvImage ret;

    if ( !image.empty() ) {
        auto maxSize = std::max( image.width(), image.height() );

        if ( maxSize != 0 ) {
            double scaleFactor = static_cast<double>(size) / static_cast<double>(maxSize);

            cv::resize( image, ret, cv::Size(), scaleFactor, scaleFactor );

        }

    }

    return ret;

}
