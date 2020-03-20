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

    CvImage undistort( const CvImage &image ) const;

    const MonocularCalibrationDataShort &calibration() const;

protected:
    MonocularCalibrationDataShort m_calibrationData;

    cv::Mat m_rMap1;
    cv::Mat m_rMap2;

    void calcRectificationMaps();

};

class StereoRectificationProcessor : public RectificationProcessorBase
{
public:
    StereoRectificationProcessor();
    StereoRectificationProcessor( const StereoCalibrationDataShort &calibrationData );

    void setCalibrationData( const StereoCalibrationDataShort &calibrationData );
    const StereoCalibrationDataShort &calibration() const;

    bool rectifyLeft( const CvImage &image, CvImage *result ) const;
    bool rectifyRight( const CvImage &image , CvImage *result ) const;

    bool cropLeft( const CvImage &image, CvImage *result ) const;
    bool cropRight( const CvImage &image , CvImage *result ) const;

    bool rectify( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult ) const;
    bool crop( const CvImage &leftImage, const CvImage &rightImage, CvImage *leftResult, CvImage *rightResult ) const;

    bool isValid() const;

protected:
    StereoCalibrationDataShort m_calibrationData;

    cv::Mat m_leftRMap1;
    cv::Mat m_leftRMap2;

    cv::Mat m_rightRMap1;
    cv::Mat m_rightRMap2;

    void calcRectificationMaps();

};
