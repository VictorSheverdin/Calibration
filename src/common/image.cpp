#include "precompiled.h"

#include "image.h"

// QtImage
QtImage::QtImage()
    : QImage()
{
}

QtImage::QtImage( const QSize &size, Format format )
    : QImage( size, format )
{
}

QtImage::QtImage( int width, int height, Format format )
    : QImage( width, height, format )
{
}

QtImage::QtImage( const QImage &img )
    : QImage( img )
{
}

QtImage::QtImage( const CvImage &img )
{
    CvImage convertedImage;

    if ( img.channels() == 3 ) {
        cv::cvtColor( img, convertedImage, cv::COLOR_BGR2RGB );

        operator=( QImage( reinterpret_cast<const unsigned char *>( convertedImage.data ),
                        convertedImage.width(), convertedImage.height(), convertedImage.step,
                        QImage::Format_RGB888 ).copy() );

    }
    else if ( img.channels() == 1 ) {
        cv::cvtColor( img, convertedImage, cv::COLOR_GRAY2RGB );

        operator=( QImage( reinterpret_cast<const unsigned char *>( convertedImage.data ),
                        convertedImage.width(), convertedImage.height(), convertedImage.step,
                        QImage::Format_RGB888 ).copy() );

//        operator=( QImage(reinterpret_cast<const unsigned char *>( img.data ), img.cols, img.rows, img.step, QImage::Format_Grayscale8 ).copy() );
    }

}

// CvImage
CvImage::CvImage()
    : cv::Mat()
{
}

CvImage::CvImage( const std::string &fileName )
    : cv::Mat( cv::imread( fileName, cv::IMREAD_UNCHANGED ) )
{
}

CvImage::CvImage(int width, int height, int type)
    : cv::Mat( height, width, type )
{
}

CvImage::CvImage(cv::Size size, int type)
    : cv::Mat( size, type )
{
}

CvImage::CvImage( int width, int height, int type, const cv::Scalar& color )
    : cv::Mat( width, height, type, color )
{
}

CvImage::CvImage( cv::Size size, int type, const cv::Scalar& color )
    : cv::Mat( size, type, color )
{
}

CvImage::CvImage( int rows, int cols, int type, void* data, size_t step )
    : cv::Mat( rows, cols, type, data, step )
{
}

CvImage::CvImage( const cv::Mat &mat )
    : cv::Mat( mat )
{
}

CvImage::CvImage( const QtImage &img )
    : cv::Mat()
{
    if ( img.format() == QImage::Format_RGB888 ) {

        QtImage cloneImage = img.copy();

        CvImage cvImage;

        cv::cvtColor( cv::Mat( cloneImage.height(), cloneImage.width(), CV_8UC3, cloneImage.bits(), cloneImage.bytesPerLine() ).clone(), cvImage, cv::COLOR_RGB2BGR );

        operator=( cvImage );

    }
    else if ( img.format() == QImage::Format_Indexed8 ) {

        QtImage cloneImage = img.copy();

        operator=( CvImage( cloneImage.height(), cloneImage.width(), CV_8U, cloneImage.bits(), cloneImage.bytesPerLine() ).clone() );

    }

}

int CvImage::width() const
{
    return cols;
}

int CvImage::height() const
{
    return rows;
}

cv::Size CvImage::size() const
{
    return cv::Size( width(), height() );
}

double CvImage::aspectRatio() const
{
    if ( height() != 0 )
        return static_cast<double>( width() ) / height();
    else
        return 0.0;

}

double CvImage::revAspectRatio() const
{
    if ( width() != 0 )
        return static_cast<double>( height() ) / width();
    else
        return 0.0;

}

// StereoImage
StereoImage::StereoImage()
{
}

StereoImage::StereoImage( const CvImage &leftImage, const CvImage &rightImage )
{
    setLeftImage( leftImage );
    setRightImage( rightImage );
}

const CvImage &StereoImage::leftImage() const
{
    return m_leftImage;
}

void StereoImage::setLeftImage( const CvImage &img )
{
    m_leftImage = img;
}

const CvImage &StereoImage::rightImage() const
{
    return m_rightImage;
}

void StereoImage::setRightImage( const CvImage &img )
{
    m_rightImage = img;
}

bool StereoImage::empty() const
{
    return m_leftImage.empty() || m_rightImage.empty();
}

// StampedImageBase
StampedImageBase::StampedImageBase( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    setTime( time );
}

void StampedImageBase::setTime( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    m_time = time;
}

const std::chrono::time_point< std::chrono::system_clock > &StampedImageBase::time() const
{
    return m_time;
}

// StampedImage
StampedImage::StampedImage( const std::chrono::time_point<std::chrono::system_clock> &time )
    : StampedImageBase( time )
{
}

StampedImage::StampedImage( const std::chrono::time_point< std::chrono::system_clock > &time, const CvImage &mat )
    : StampedImageBase( time ), CvImage( mat )
{
}

StampedImage::StampedImage( const std::chrono::time_point< std::chrono::system_clock > &time, const cv::Mat &mat )
    : StampedImageBase( time ), CvImage( mat )
{
}

StampedImage::StampedImage( const std::chrono::time_point< std::chrono::system_clock > &time, const QtImage &img )
    : StampedImageBase( time ), CvImage( img )
{
}

StampedImage::StampedImage( const CvImage &img )
    : CvImage( img )
{
}

StampedImage::StampedImage( const cv::Mat &mat )
    : CvImage( mat )
{
}

StampedImage::StampedImage( const QtImage &img )
    : CvImage( img )
{
}

int64_t StampedImage::diffMs( const StampedImage &other ) const
{
    return std::abs( std::chrono::duration_cast< std::chrono::microseconds >( m_time - other.m_time ).count() );
}

// StampedStereoImage
StampedStereoImage::StampedStereoImage()
{
}

StampedStereoImage::StampedStereoImage( const StampedImage &leftImage, const StampedImage &rightImage )
{
    set( leftImage, rightImage );
}

StampedStereoImage::StampedStereoImage( const StereoImage &image )
{
    set( image );
}

void StampedStereoImage::set( const StampedImage &leftImage, const StampedImage &rightImage )
{
    setLeftImage( leftImage );
    setRightImage( rightImage );
}

void StampedStereoImage::set( const StereoImage &image )
{
    setLeftImage( image.leftImage() );
    setRightImage( image.rightImage() );
}

void StampedStereoImage::setLeftImage( const StampedImage &image )
{
    m_leftImage = image;
}

void StampedStereoImage::setRightImage( const StampedImage &image )
{
    m_rightImage = image;
}

const StampedImage &StampedStereoImage::leftImage() const
{
    return m_leftImage;
}

const StampedImage &StampedStereoImage::rightImage() const
{
    return m_rightImage;
}

int StampedStereoImage::diffMs() const
{
    return m_leftImage.diffMs( m_rightImage );
}

bool StampedStereoImage::empty() const
{
    return m_leftImage.empty() || m_rightImage.empty();
}

StampedStereoImage::operator StereoImage()
{
    return StereoImage( m_leftImage, m_rightImage );
}
