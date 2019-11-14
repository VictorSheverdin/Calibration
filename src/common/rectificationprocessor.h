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

    const MonocularCalibrationDataShort &calibration() const;

protected:
    MonocularCalibrationDataShort m_calibrationData;

};

class StereoRectificationProcessor : public RectificationProcessorBase
{
public:
    StereoRectificationProcessor();
    StereoRectificationProcessor( const StereoCalibrationDataShort &calibrationData );
    StereoRectificationProcessor( const std::string &fileName );

    bool rectifyLeft( const CvImage &image, CvImage *result );
    bool rectifyRight( const CvImage &image , CvImage *result );

    bool rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult );

    bool isValid() const;

    const StereoCalibrationDataShort &calibration() const;

protected:
    StereoCalibrationDataShort m_calibrationData;

    static cv::Rect cropRect( const cv::Rect &leftCropRect, const cv::Rect &rightCropRect );

};
