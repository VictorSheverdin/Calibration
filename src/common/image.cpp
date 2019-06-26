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
        cv::cvtColor( img, convertedImage, CV_BGR2RGB );

        operator=( QImage( reinterpret_cast<const unsigned char *>( convertedImage.data ),
                        convertedImage.width(), convertedImage.height(), convertedImage.step,
                        QImage::Format_RGB888 ).copy() );

    }
    else if ( img.channels() == 1 ) {
        cv::cvtColor( img, convertedImage, CV_GRAY2RGB );

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

CvImage::CvImage( const QString &fileName )
    : cv::Mat( cv::imread( fileName.toStdString() ) )
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

        cv::cvtColor( cv::Mat( cloneImage.height(), cloneImage.width(), CV_8UC3, cloneImage.bits(), cloneImage.bytesPerLine() ).clone(), cvImage, CV_RGB2BGR );

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

// Frame
Frame::Frame()
    : CvImage()
{
    initialize();
}

Frame::Frame( const CvImage &img )
    : CvImage( img )
{
    initialize();
}

Frame::Frame( const cv::Mat &mat )
    : CvImage( mat )
{
    initialize();
}

Frame::Frame( const QtImage &img )
    : CvImage( img )
{
    initialize();
}

void Frame::initialize()
{
    m_time = std::chrono::system_clock::now();
}

const std::chrono::time_point< std::chrono::system_clock > &Frame::time() const
{
    return m_time;
}

int Frame::timeDiff( const Frame &other ) const
{
    return std::abs( std::chrono::duration_cast<std::chrono::milliseconds>( m_time - other.m_time ).count() );
}

