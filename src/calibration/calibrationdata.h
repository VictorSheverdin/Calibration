#pragma once

#include "src/common/calibrationdatabase.h"

class MonocularIcon;
class StereoIcon;

class MonocularCalibrationResult
{
public:
    MonocularCalibrationResult();

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
    cv::Mat m_rVec;
    cv::Mat m_tVec;

    std::vector< cv::Point2f > m_points2d;
    std::vector< cv::Point3f > m_points3d;

    bool m_ok;

private:
    void initialize();

};

class MonocularCalibrationData : public MonocularCalibrationDataShort
{
public:
    MonocularCalibrationData();

    void setResults( std::vector< MonocularCalibrationResult > &value );
    const std::vector< MonocularCalibrationResult > &results() const;

    MonocularCalibrationResult &result( const unsigned int i );
    const MonocularCalibrationResult &result( const unsigned int i ) const;

    const unsigned int resultsSize() const;

    void setPreviewImage( const CvImage &image );
    const CvImage &previewImage() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

protected:
    std::vector< MonocularCalibrationResult > m_results;

    CvImage m_previewImage;

private:
    void initialize();
};

class StereoCalibrationData : public StereoCalibrationDataBase
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

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

protected:
    MonocularCalibrationData m_leftCameraResults;
    MonocularCalibrationData m_rightCameraResults;

};


