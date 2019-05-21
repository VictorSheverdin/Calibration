#pragma once

#include "image.h"

class MonocularCalibrationResult
{
public:
    MonocularCalibrationResult();

    void setSourceView( const CvImage &value );
    const CvImage &sourceView() const;

    void setProcessedView( const CvImage &value );
    const CvImage &processedView() const;

    void setRVec( const cv::Mat &value );
    const cv::Mat &rVec() const;

    void setTVec( const cv::Mat &value );
    const cv::Mat &tVec() const;

    void setPoints2d( const std::vector< cv::Point2f > &value );
    const std::vector< cv::Point2f > &points2d() const;

    void setPoints3d( const std::vector< cv::Point3f > &value );
    const std::vector< cv::Point3f > &points3d() const;

    void setOk( const bool value );
    bool isOk() const;

protected:
    CvImage m_sourceView;
    CvImage m_processedView;
    cv::Mat m_rVec;
    cv::Mat m_tVec;

    std::vector< cv::Point2f > m_points2d;
    std::vector< cv::Point3f > m_points3d;

    bool m_ok;

private:
    void initialize();

};

class MonocularCalibrationData
{
public:
    MonocularCalibrationData();

    void setFrameSize( const cv::Size &value );
    const cv::Size &frameSize() const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void setDistortionCoefficients( const cv::Mat &value );
    const cv::Mat &distortionCoefficients() const;

    void setResults( std::vector< MonocularCalibrationResult > &value );
    const std::vector< MonocularCalibrationResult > &results() const;

    MonocularCalibrationResult &result( const unsigned int i );
    const MonocularCalibrationResult &result( const unsigned int i ) const;

    void setOk( const bool value );
    bool isOk() const;

    void setError( const double value );
    double error() const;

    const unsigned int resultsSize() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

protected:
    cv::Size m_frameSize;

    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoefficients;

    std::vector< MonocularCalibrationResult > m_results;

    bool m_ok;

    double m_error;

private:
    void initialize();
};

class StereoCalibrationData
{
public:
    StereoCalibrationData();

    void setLeftCameraResults( const MonocularCalibrationData &value );
    const MonocularCalibrationData &leftCameraResults() const;
    MonocularCalibrationData &leftCameraResults();

    void setRightCameraResults( const MonocularCalibrationData &value );
    const MonocularCalibrationData &rightCameraResults() const;
    MonocularCalibrationData &rightCameraResults();

    unsigned int leftResultsSize() const;
    unsigned int rightResultsSize() const;

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

    void setError( const double value );
    double error() const;

    bool isOk() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

protected:
    MonocularCalibrationData m_leftCameraResults;
    MonocularCalibrationData m_rightCameraResults;

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

    double m_error;

};


