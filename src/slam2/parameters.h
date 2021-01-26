#pragma once

#include <opencv2/opencv.hpp>

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

class StereoCameraMatrix
{
public:
    StereoCameraMatrix() = default;
    StereoCameraMatrix( const cv::Mat &left, const cv::Mat &right );

    void set( const cv::Mat &left, const cv::Mat &right );

    const cv::Mat &left() const;
    const cv::Mat &right() const;

protected:
    cv::Mat _left;
    cv::Mat _right;

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

    void setProcessRect( const StereoRect &rect );

    void setCameraMatrix( const cv::Mat &left, const cv::Mat &right );
    void setDistCoefficients( const cv::Mat &left, const cv::Mat &right );

    StereoRect processRect() const;

    const StereoCameraMatrix &cameraMatrix() const;
    const StereoDistorsionCoefficients &distorsionCoefficients() const;

    void setFlowTracker( const std::shared_ptr< FlowTracker > &value );
    void setFeatureTracker( const std::shared_ptr< FeatureTracker > &value );

    const std::shared_ptr< FlowTracker > &flowTracker() const;
    const std::shared_ptr< FeatureTracker > &featureTracker() const;

    void setCornerExtractionCount( const size_t value );
    size_t cornerExtractionCount() const;

protected:
    cv::Size _frameSize;

    StereoRect _procRect;

    StereoCameraMatrix _cameraMatrix;
    StereoDistorsionCoefficients _distorsionCoefficients;

    std::shared_ptr< FlowTracker > _flowTracker;
    std::shared_ptr< FeatureTracker > _featureTracker;

    size_t _cornerExtractionCount;

private:
    void initialize();

};

}