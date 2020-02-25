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

    void setCalibrationData( const MonocularCalibrationDataShort &calibrationData );
    void loadFile( const std::string &fileName );

    CvImage undistort( const CvImage &image );

    const MonocularCalibrationDataShort &calibration() const;

protected:
    MonocularCalibrationDataShort m_calibrationData;

};

class StereoRectificationProcessor : public RectificationProcessorBase
{
public:
    StereoRectificationProcessor();
    StereoRectificationProcessor( const StereoCalibrationDataBase &calibrationData );

    void setCalibrationData( const StereoCalibrationDataBase &calibrationData );
    const StereoCalibrationDataBase &calibration() const;

    bool rectifyLeft( const CvImage &image, CvImage *result );
    bool rectifyRight( const CvImage &image , CvImage *result );

    bool cropLeft( const CvImage &image, CvImage *result );
    bool cropRight( const CvImage &image , CvImage *result );

    bool rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult );
    bool crop( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult );

    bool isValid() const;

protected:
    StereoCalibrationDataBase m_calibrationData;

};
