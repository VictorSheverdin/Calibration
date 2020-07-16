#pragma once

#include <QImage>

#include <opencv2/opencv.hpp>

#include <chrono>

class CvImage;

class QtImage : public QImage
{
public:
    QtImage();
    QtImage( const QSize &size, Format format = Format_RGBA8888 );
    QtImage( int width, int height, Format format = Format_RGBA8888 );
    QtImage( const QImage &img );
    QtImage( const CvImage &img );
};

class CvImage : public cv::Mat
{
public:
    CvImage();
    CvImage( const std::string &fileName );
    CvImage( int width, int height, int type );
    CvImage( cv::Size size, int type );
    CvImage( int width, int height, int type, const cv::Scalar& color );
    CvImage( cv::Size size, int type, const cv::Scalar& color );
    CvImage( int rows, int cols, int type, void* data, size_t step=AUTO_STEP );
    CvImage( const cv::Mat &mat );
    CvImage( const QtImage &img );

    int width() const;
    int height() const;

    cv::Size size() const;

    double aspectRatio() const;
    double revAspectRatio() const;
};

Q_DECLARE_METATYPE( CvImage )

class StereoImage
{
public:
    StereoImage();
    StereoImage( const CvImage &leftImage, const CvImage &rightImage );

    const CvImage &leftImage() const;
    void setLeftImage( const CvImage &img );

    const CvImage &rightImage() const;
    void setRightImage( const CvImage &img );

    bool empty() const;

protected:
    CvImage m_leftImage;
    CvImage m_rightImage;
};

class StampedImageBase
{
public:
    StampedImageBase( const std::chrono::time_point< std::chrono::system_clock > &time = std::chrono::system_clock::now() );

    void setTime( const std::chrono::time_point< std::chrono::system_clock > &time );
    const std::chrono::time_point< std::chrono::system_clock > &time() const;

protected:
    std::chrono::time_point< std::chrono::system_clock > m_time;
};

class StampedImage : public StampedImageBase, public CvImage
{
public:
    StampedImage( const std::chrono::time_point< std::chrono::system_clock > &time = std::chrono::system_clock::now() );

    StampedImage( const std::chrono::time_point< std::chrono::system_clock > &time, const CvImage &mat );
    StampedImage( const std::chrono::time_point< std::chrono::system_clock > &time, const cv::Mat &mat );
    StampedImage( const std::chrono::time_point< std::chrono::system_clock > &time, const QtImage &img );

    StampedImage( const CvImage &mat );
    StampedImage( const cv::Mat &mat );
    StampedImage( const QtImage &img );

    int64_t diffMs( const StampedImage &other ) const;
};

class StampedStereoImage
{
public:
    StampedStereoImage();

    StampedStereoImage( const StampedImage &leftImage, const StampedImage &rightImage );
    StampedStereoImage( const StereoImage &image );

    const StampedImage &leftImage() const;
    void setLeftImage( const StampedImage &image );

    const StampedImage &rightImage() const;
    void setRightImage( const StampedImage &image );

    int diffMs() const;

    bool empty() const;

    operator StereoImage();

protected:
    StampedImage m_leftImage;
    StampedImage m_rightImage;
};
