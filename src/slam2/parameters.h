#pragma once

#include <opencv2/opencv.hpp>

#include "src/common/calibrationdatabase.h"
namespace slam2 {

class FlowTracker;
class FeatureTracker;

class StereoRect
{
public:
    StereoRect() = default;
    StereoRect( const cv::Rect &leftRect, const cv::Rect &rightRect );

    void set( const cv::Rect &leftRect, const cv::Rect &rightRect );

    const cv::Rect &left() const;
    const cv::Rect &right() const;

protected:
    cv::Rect _left;
    cv::Rect _right;

};

class StereoMat
{
public:
    StereoMat() = default;
    StereoMat( const cv::Mat &left, const cv::Mat &right );

    void set( const cv::Mat &left, const cv::Mat &right );

    const cv::Mat &left() const;
    const cv::Mat &right() const;

protected:
    cv::Mat _left;
    cv::Mat _right;

};

class StereoCameraMatrix : public StereoMat
{
public:
    StereoCameraMatrix() = default;
    StereoCameraMatrix( const cv::Mat &left, const cv::Mat &right );

};

class StereoDistorsionCoefficients
{
public:
    StereoDistorsionCoefficients() = default;
    StereoDistorsionCoefficients( const cv::Mat &left, const cv::Mat &right );

    void set( const cv::Mat &left, const cv::Mat &right );

    const cv::Mat &left() const;
    const cv::Mat &right() const;

protected:
    cv::Mat _left;
    cv::Mat _right;

};

class Parameters
{
public:
    Parameters();

    void setCalibration( const StereoCalibrationDataShort &value );
    const StereoCalibrationDataShort &calibration() const;

    void setMask( const StereoMat &value );
    const StereoMat &mask() const;

    StereoRect processRect() const;
    void setProcessRect( const StereoRect &rect );

    StereoCameraMatrix cameraMatrix() const;
    StereoDistorsionCoefficients distorsionCoefficients() const;

    void setFlowTracker( const std::shared_ptr< FlowTracker > &value );
    const std::shared_ptr< FlowTracker > &flowTracker() const;

    void setFeatureTracker( const std::shared_ptr< FeatureTracker > &value );
    const std::shared_ptr< FeatureTracker > &featureTracker() const;

    void setCornerExtractionCount( const size_t value );
    size_t cornerExtractionCount() const;

    void setMinimumTracksCount(  const size_t value );
    size_t minimumTracksCount() const;

    cv::Mat rightRotation() const;
    cv::Mat rightTranslation() const;

    double maxReprojectionError() const;

    size_t minimumRecoverPointsCount() const;

    double minimumInliersRatio() const;

    double extractionDistance() const;

    void setMinimumTriangulationDistance( const double value );
    double minimumTriangulationDistance() const;

    double minimumDisparity() const;

    double pointsDrawScale() const;
    double textDrawScale() const;

protected:
    StereoCalibrationDataShort _calibration;

    StereoMat _mask;

    StereoRect _procRect;

    std::shared_ptr< FlowTracker > _flowTracker;
    std::shared_ptr< FeatureTracker > _featureTracker;

    size_t _cornerExtractionCount;

    size_t _minimumTracksCount;

    double _extractionDistance;

    double _minimumTriangulationDistance;

    size_t _minimumRecoverPointsCount;

    double _minimumInliersRatio;

    double _minimumDisparity;

    double _maxReprojectionError;

private:
    void initialize();

};

}
