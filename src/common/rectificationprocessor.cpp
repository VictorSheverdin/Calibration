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
{
    setCalibrationData( calibrationData );
}

MonoUndistortionProcessor::MonoUndistortionProcessor( const std::string &fileName )
{
    loadFile( fileName );
}

void MonoUndistortionProcessor::setCalibrationData( const MonocularCalibrationDataShort &calibrationData )
{
    m_calibrationData = calibrationData;
}

void MonoUndistortionProcessor::loadFile( const std::string &fileName )
{
    m_calibrationData.loadYaml( fileName );
}

CvImage MonoUndistortionProcessor::undistort( const CvImage &image )
{
    CvImage ret;

    cv::undistort( image, ret, m_calibrationData.cameraMatrix(), m_calibrationData.distortionCoefficients() );

    return ret;

}

const MonocularCalibrationDataShort &MonoUndistortionProcessor::calibration() const
{
    return m_calibrationData;
}

// StereoRectificationProcessor
StereoRectificationProcessor::StereoRectificationProcessor()
{
}

StereoRectificationProcessor::StereoRectificationProcessor( const StereoCalibrationDataShort &calibrationData )
{
    setCalibrationData( calibrationData );
}

StereoRectificationProcessor::StereoRectificationProcessor( const std::string &fileName )
{
    loadFile( fileName );
}

void StereoRectificationProcessor::setCalibrationData( const StereoCalibrationDataShort &calibrationData )
{
    m_calibrationData = calibrationData;
}

void StereoRectificationProcessor::loadFile( const std::string &fileName )
{
    m_calibrationData.loadYaml( fileName );
}

bool StereoRectificationProcessor::rectifyLeft( const CvImage &image , CvImage *result )
{
    if ( !result || !isValid() )
        return false;

    cv::remap( image, *result, m_calibrationData.leftRMap(), m_calibrationData.leftDMap(), cv::INTER_CUBIC );

    //if (!m_calibrationData.leftROI().empty() && !m_calibrationData.rightROI().empty())
    //    *result = (*result)( cropRect( m_calibrationData.leftROI(), m_calibrationData.rightROI() ) );

    return true;

}

bool StereoRectificationProcessor::rectifyRight( const CvImage &image, CvImage *result )
{
    if ( !result || !isValid() )
        return false;

    cv::remap( image, *result, m_calibrationData.rightRMap(), m_calibrationData.rightDMap(), cv::INTER_CUBIC );

    //if ( !m_calibrationData.leftROI().empty() && !m_calibrationData.rightROI().empty())
    //    *result = (*result)( cropRect( m_calibrationData.leftROI(), m_calibrationData.rightROI() ) );

    return true;

}

bool StereoRectificationProcessor::rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult )
{
    if ( !rectifyLeft( leftImage, leftResult ) )
        return false;

    if ( !rectifyRight( rightImage, rightResult ) )
        return false;

    return true;

}

bool StereoRectificationProcessor::isValid() const
{
    return m_calibrationData.isOk();
}

const StereoCalibrationDataShort &StereoRectificationProcessor::calibration() const
{
    return m_calibrationData;
}

cv::Rect StereoRectificationProcessor::cropRect( const cv::Rect &leftCropRect, const cv::Rect &rightCropRect )
{
    cv::Rect ret;

    ret.x = std::max( leftCropRect.x, rightCropRect.x );
    ret.y = std::max( leftCropRect.y, rightCropRect.y );
    ret.width = std::min( leftCropRect.x + leftCropRect.width, rightCropRect.x + rightCropRect.width ) - ret.x;
    ret.height = std::min( leftCropRect.y + leftCropRect.height, rightCropRect.y + rightCropRect.height ) - ret.y;

    return ret;
}
