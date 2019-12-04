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

    return true;

}

bool StereoRectificationProcessor::rectifyRight( const CvImage &image, CvImage *result )
{
    if ( !result || !isValid() )
        return false;

    cv::remap( image, *result, m_calibrationData.rightRMap(), m_calibrationData.rightDMap(), cv::INTER_CUBIC );

    return true;

}

bool StereoRectificationProcessor::cropLeft( const CvImage &image, CvImage *result )
{
    if ( !result || !isValid() )
        return false;

    auto cropRect = m_calibrationData.cropRect();

    if ( cropRect.empty() )
        return false;

    *result = image( m_calibrationData.cropRect() );
    return true;

}

bool StereoRectificationProcessor::cropRight( const CvImage &image , CvImage *result )
{
    if ( !result || !isValid() )
        return false;

    if ( m_calibrationData.leftROI().empty() || m_calibrationData.rightROI().empty() )
        return false;

    *result = image( m_calibrationData.cropRect() );
    return true;

}

bool StereoRectificationProcessor::rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult )
{
    auto result = rectifyLeft( leftImage, leftResult );
    result = result && rectifyRight( rightImage, rightResult );

    return result;
}

bool StereoRectificationProcessor::crop( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult )
{
    auto result = cropLeft( leftImage, leftResult );
    result = result && cropRight( rightImage, rightResult );

    return result;
}

bool StereoRectificationProcessor::isValid() const
{
    return m_calibrationData.isOk();
}

const StereoCalibrationDataShort &StereoRectificationProcessor::calibration() const
{
    return m_calibrationData;
}
