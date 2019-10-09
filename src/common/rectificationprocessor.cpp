#include "precompiled.h"

#include "rectificationprocessor.h"

// RectificationProcessorBase
RectificationProcessorBase::RectificationProcessorBase()
{
}

// MonoUndistortionProcessor
MonoUndistortionProcessor::MonoUndistortionProcessor()
{
}

MonoUndistortionProcessor::MonoUndistortionProcessor( const MonocularCalibrationDataShort &calibrationData )
    : m_calibrationData( calibrationData )
{
}

MonoUndistortionProcessor::MonoUndistortionProcessor( const std::string &fileName )
{
    m_calibrationData.loadYaml( fileName );
}


CvImage MonoUndistortionProcessor::undistort( const CvImage &image )
{
    CvImage ret;

    cv::undistort( image, ret, m_calibrationData.cameraMatrix(), m_calibrationData.distortionCoefficients() );

    return ret;

}

// StereoRectificationProcessor
StereoRectificationProcessor::StereoRectificationProcessor()
{
}

StereoRectificationProcessor::StereoRectificationProcessor( const StereoCalibrationDataShort &calibrationData )
    : m_calibrationData( calibrationData )
{
}

StereoRectificationProcessor::StereoRectificationProcessor( const std::string &fileName )
{
    m_calibrationData.loadYaml( fileName );
}

CvImage StereoRectificationProcessor::rectifyLeft( const CvImage &image )
{
    CvImage ret;

    cv::remap( image, ret, m_calibrationData.leftRMap(), m_calibrationData.leftDMap(), cv::INTER_LANCZOS4 );

    return ret;

}

CvImage StereoRectificationProcessor::rectifyRight( const CvImage &image )
{
    CvImage ret;

    cv::remap( image, ret, m_calibrationData.rightRMap(), m_calibrationData.rightDMap(), cv::INTER_LANCZOS4 );

    return ret;

}

void StereoRectificationProcessor::rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult )
{
    if ( leftResult )
        *leftResult = rectifyLeft( leftImage );

    if ( rightResult )
        *rightResult = rectifyRight( rightImage );

}
