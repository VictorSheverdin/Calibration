#pragma once

#include "src/common/image.h"

#include <opencv/cv.hpp>

class BMProcessor
{
public:
    BMProcessor();

    int getMinDisparity() const;
    void setMinDisparity( const int minDisparity );

    int getNumDisparities() const;
    void setNumDisparities( const int numDisparities );

    int getBlockSize() const;
    void setBlockSize( const int blockSize );

    int getSpeckleWindowSize() const;
    void setSpeckleWindowSize( const int speckleWindowSize );

    int getSpeckleRange() const;
    void setSpeckleRange( const int speckleRange );

    int getDisp12MaxDiff() const;
    void setDisp12MaxDiff( const int disp12MaxDiff );

    int getPreFilterType() const;
    void setPreFilterType( const int preFilterType );

    int getPreFilterSize() const;
    void setPreFilterSize( const int preFilterSize );

    int getPreFilterCap() const;
    void setPreFilterCap( const int preFilterCap );

    int getTextureThreshold() const;
    void setTextureThreshold( const int textureThreshold );

    int getUniquenessRatio() const;
    void setUniquenessRatio( const int uniquenessRatio );

    cv::Rect getROI1() const;
    void setROI1( const cv::Rect &roi1 );

    cv::Rect getROI2() const;
    void setROI2( const cv::Rect &roi2 );

    cv::Mat processDisparity( const CvImage &left, const CvImage &right );

protected:
    cv::Ptr< cv::StereoBM > m_matcher;

private:
    void initialize();

};

class GMProcessor
{
public:
    GMProcessor();

    int getMode() const;
    void setMode(int mode);

    int getMinDisparity() const;
    void setMinDisparity( const int minDisparity );

    int getNumDisparities() const;
    void setNumDisparities( const int numDisparities );

    int getBlockSize() const;
    void setBlockSize( const int blockSize );

    int getSpeckleWindowSize() const;
    void setSpeckleWindowSize( const int speckleWindowSize );

    int getSpeckleRange() const;
    void setSpeckleRange( const int speckleRange );

    int getDisp12MaxDiff() const;
    void setDisp12MaxDiff( const int disp12MaxDiff );

    int getPreFilterCap() const;
    void setPreFilterCap( const int preFilterCap );

    int getUniquenessRatio() const;
    void setUniquenessRatio( const int uniquenessRatio );

    int getP1() const;
    void setP1( int p1 );

    int getP2() const;
    void setP2( int p2 );

    cv::Mat processDisparity( const CvImage &left, const CvImage &right );

protected:
    cv::Ptr< cv::StereoSGBM > m_matcher;

private:
    void initialize();

};
