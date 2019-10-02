#pragma once

#include "src/common/image.h"

class CalibrationDataBase
{
public:
    CalibrationDataBase();

    void setOk( const bool value );
    bool isOk() const;

    void setError( const double value );
    double error() const;

protected:
    bool m_ok;
    double m_error;

private:
    void initialize();
};

class MonocularCalibrationDataShort : public CalibrationDataBase
{
public:
    MonocularCalibrationDataShort();
    MonocularCalibrationDataShort( const std::string &fileName );

    void setFrameSize( const cv::Size &value );
    const cv::Size &frameSize() const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void setDistortionCoefficients( const cv::Mat &value );
    const cv::Mat &distortionCoefficients() const;

    int frameWidth() const;
    int frameHeight() const;

    void setFx( const double value );
    void setFy( const double value );
    void setCx( const double value );
    void setCy( const double value );

    double fx() const;
    double fy() const;
    double cx() const;
    double cy() const;

    void shiftPrincipalValues( const cv::Point2i &value );

    double k1() const;
    double k2() const;
    double k3() const;
    double k4() const;
    double k5() const;
    double k6() const;
    double p1() const;
    double p2() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

protected:
    cv::Size m_frameSize;

    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoefficients;

private:
    void initialize();
};

class StereoCalibrationDataBase : public CalibrationDataBase
{
public:
    StereoCalibrationDataBase();

    void setCorrespondFrameCount( const unsigned int value );
    unsigned int correspondFrameCount() const;

    void setRotationMatrix( const cv::Mat &value );
    const cv::Mat &rotationMatrix() const;

    void setTranslationVector( const cv::Mat &value );
    const cv::Mat &translationVector() const;

    void setFundamentalMatrix( const cv::Mat &value );
    const cv::Mat &fundamentalMatrix() const;

    void setEssentialMatrix( const cv::Mat &value );
    const cv::Mat &essentialMatrix() const;

    void setLeftRectifyMatrix( const cv::Mat &value );
    const cv::Mat &leftRectifyMatrix() const;

    void setRightRectifyMatrix( const cv::Mat &value );
    const cv::Mat &rightRectifyMatrix() const;

    void setLeftProjectionMatrix( const cv::Mat &value );
    const cv::Mat &leftProjectionMatrix() const;

    void setRightProjectionMatrix( const cv::Mat &value );
    const cv::Mat &rightProjectionMatrix() const;

    void setDisparityToDepthMatrix( const cv::Mat &value );
    const cv::Mat &disparityToDepthMatrix() const;

    void setLeftROI( const cv::Rect &value );
    const cv::Rect &leftROI() const;

    void setRightROI( const cv::Rect &value );
    const cv::Rect &rightROI() const;

    void setLeftRMap( const cv::Mat &value );
    const cv::Mat &leftRMap() const;

    void setLeftDMap( const cv::Mat &value );
    const cv::Mat &leftDMap() const;

    void setRightRMap( const cv::Mat &value );
    const cv::Mat &rightRMap() const;

    void setRightDMap( const cv::Mat &value );
    const cv::Mat &rightDMap() const;

    double distance() const;

protected:
    unsigned int m_correspondFrameCount;

    cv::Mat m_rotationMatrix;
    cv::Mat m_translationVector;
    cv::Mat m_fundamentalMatrix;
    cv::Mat m_essentialMatrix;

    cv::Mat m_leftRectifyMatrix;
    cv::Mat m_rightRectifyMatrix;
    cv::Mat m_leftProjectionMatrix;
    cv::Mat m_rightProjectionMatrix;
    cv::Mat m_disparityToDepthMatrix;

    cv::Rect m_leftROI;
    cv::Rect m_rightROI;

    cv::Mat m_leftRMap;
    cv::Mat m_leftDMap;
    cv::Mat m_rightRMap;
    cv::Mat m_rightDMap;

private:
    void initialize();

};

class StereoCalibrationDataShort : public StereoCalibrationDataBase
{
public:
    StereoCalibrationDataShort();
    StereoCalibrationDataShort( const std::string &fileName );

    void setLeftCameraResults( const MonocularCalibrationDataShort &value );
    const MonocularCalibrationDataShort &leftCameraResults() const;
    MonocularCalibrationDataShort &leftCameraResults();

    void setRightCameraResults( const MonocularCalibrationDataShort &value );
    const MonocularCalibrationDataShort &rightCameraResults() const;
    MonocularCalibrationDataShort &rightCameraResults();

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

protected:
    MonocularCalibrationDataShort m_leftCameraResults;
    MonocularCalibrationDataShort m_rightCameraResults;

private:
    void initialize();

};



