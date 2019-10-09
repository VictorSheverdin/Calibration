#pragma once

#include "calibrationdatabase.h"

class RectificationProcessorBase
{
public:
    RectificationProcessorBase();

private:
};

class MonoUndistortionProcessor : public RectificationProcessorBase
{
public:
    MonoUndistortionProcessor();
    MonoUndistortionProcessor( const MonocularCalibrationDataShort &calibrationData );
    MonoUndistortionProcessor( const std::string &fileName );

    CvImage undistort( const CvImage &image );

protected:
    MonocularCalibrationDataShort m_calibrationData;

};

class StereoRectificationProcessor : public RectificationProcessorBase
{
public:
    StereoRectificationProcessor();
    StereoRectificationProcessor( const StereoCalibrationDataShort &calibrationData );
    StereoRectificationProcessor( const std::string &fileName );

    CvImage rectifyLeft( const CvImage &image );
    CvImage rectifyRight( const CvImage &image );

    void rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult );

protected:
    StereoCalibrationDataShort m_calibrationData;

};
