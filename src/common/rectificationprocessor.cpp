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

    calcRectificationMaps();
}

void MonoUndistortionProcessor::loadFile( const std::string &fileName )
{
    m_calibrationData.loadYaml( fileName );
}

void MonoUndistortionProcessor::calcRectificationMaps()
{
    cv::initUndistortRectifyMap( m_calibrationData.cameraMatrix(), m_calibrationData.distortionCoefficients(),
                                 cv::Mat(), m_calibrationData.cameraMatrix(), m_calibrationData.frameSize(),
                                 CV_32FC2, m_rMap1, m_rMap2 );
}


CvImage MonoUndistortionProcessor::undistort( const CvImage &image ) const
{
    CvImage ret;
    cv::remap( image, ret, m_rMap1, m_rMap2, cv::INTER_CUBIC );

    // cv::undistort( image, ret, m_calibrationData.cameraMatrix(), m_calibrationData.distortionCoefficients() );

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

void StereoRectificationProcessor::setCalibrationData( const StereoCalibrationDataShort &calibrationData )
{
    m_calibrationData = calibrationData;

    calcRectificationMaps();
}

const StereoCalibrationDataShort &StereoRectificationProcessor::calibration() const
{
    return m_calibrationData;
}

bool StereoRectificationProcessor::rectifyLeft( const CvImage &image , CvImage *result ) const
{
    if ( !result || !isValid() )
        return false;

    cv::remap( image, *result, m_leftRMap1, m_leftRMap2, cv::INTER_CUBIC );

    return true;

}

bool StereoRectificationProcessor::rectifyRight( const CvImage &image, CvImage *result ) const
{
    if ( !result || !isValid() )
        return false;

    cv::remap( image, *result, m_rightRMap1, m_rightRMap2, cv::INTER_CUBIC );

    return true;

}

bool StereoRectificationProcessor::cropLeft( const CvImage &image, CvImage *result ) const
{
    if ( !result || !isValid() )
        return false;

    auto cropRect = m_calibrationData.cropRect();

    if ( cropRect.empty() )
        return false;

    *result = image( m_calibrationData.cropRect() );
    return true;

}

bool StereoRectificationProcessor::cropRight( const CvImage &image , CvImage *result ) const
{
    if ( !result || !isValid() )
        return false;

    if ( m_calibrationData.leftROI().empty() || m_calibrationData.rightROI().empty() )
        return false;

    *result = image( m_calibrationData.cropRect() );
    return true;

}

void StereoRectificationProcessor::calcRectificationMaps()
{
    cv::initUndistortRectifyMap( m_calibrationData.leftCameraResults().cameraMatrix(), m_calibrationData.leftCameraResults().distortionCoefficients(),
                                 m_calibrationData.leftRectifyMatrix(), m_calibrationData.leftProjectionMatrix().projectionMatrix(), m_calibrationData.leftCameraResults().frameSize(),
                                 CV_32FC2, m_leftRMap1, m_leftRMap2 );

    cv::initUndistortRectifyMap( m_calibrationData.rightCameraResults().cameraMatrix(), m_calibrationData.rightCameraResults().distortionCoefficients(),
                                 m_calibrationData.rightRectifyMatrix(), m_calibrationData.rightProjectionMatrix().projectionMatrix(), m_calibrationData.rightCameraResults().frameSize(),
                                 CV_32FC2, m_rightRMap1, m_rightRMap2 );

}

bool StereoRectificationProcessor::rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult ) const
{
    auto result = rectifyLeft( leftImage, leftResult );
    result = result && rectifyRight( rightImage, rightResult );

    return result;
}

bool StereoRectificationProcessor::crop( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult ) const
{
    auto result = cropLeft( leftImage, leftResult );
    result = result && cropRight( rightImage, rightResult );

    return result;
}

bool StereoRectificationProcessor::isValid() const
{
    return m_calibrationData.isOk();
}

