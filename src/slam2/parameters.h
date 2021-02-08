#pragma once

#include <opencv2/opencv.hpp>

namespace slam2 {

class Tracker;

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

    void setFrameSize( const cv::Size &value );
    const cv::Size &frameSize() const;

    StereoRect processRect() const;
    void setProcessRect( const StereoRect &rect );

    void setCameraMatrix( const cv::Mat &left, const cv::Mat &right );
    void setDistCoefficients( const cv::Mat &left, const cv::Mat &right );

    const StereoCameraMatrix &cameraMatrix() const;
    const StereoDistorsionCoefficients &distorsionCoefficients() const;

    void setTracker( const std::shared_ptr< Tracker > &value );
    const std::shared_ptr< Tracker > &tracker() const;

    void setCornerExtractionCount( const size_t value );
    size_t cornerExtractionCount() const;

    void setMinimumTracksCount(  const size_t value );
    size_t minimumTracksCount() const;

    void setRightRotation( const cv::Mat &value );
    const cv::Mat &rightRotation() const;

    void setRightTranslation( const cv::Mat &value );
    const cv::Mat &rightTranslation() const;

    double maxReprojectionError() const;

    size_t minimumRecoverPointsCount() const;

    double minimumInliersRatio() const;

    double extractionDistance() const;

    double minimumStereoDisparity() const;

    double pointsDrawScale() const;
    double textDrawScale() const;

protected:
    cv::Size _frameSize;

    StereoRect _procRect;

    StereoCameraMatrix _cameraMatrix;
    StereoDistorsionCoefficients _distorsionCoefficients;

    std::shared_ptr< Tracker > _tracker;

    cv::Mat _rightRotation;
    cv::Mat _rightTranslation;

    size_t _cornerExtractionCount;

    size_t _minimumTracksCount;

    double _extractionDistance;

    size_t _minimumRecoverPointsCount;

    double _minimumInliersRatio;

    double _minimumStereoDisparity;

private:
    void initialize();

};

}
