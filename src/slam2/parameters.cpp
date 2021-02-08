#include "src/common/precompiled.h"

#include "parameters.h"

#include "tracker.h"

namespace slam2 {

// StereoRect
StereoRect::StereoRect( const cv::Rect &leftRect, const cv::Rect &rightRect )
{
    set( leftRect, rightRect );
}

void StereoRect::set( const cv::Rect &leftRect, const cv::Rect &rightRect )
{
    _left = leftRect;
    _right = rightRect;
}

const cv::Rect &StereoRect::left() const
{
    return _left;
}

const cv::Rect &StereoRect::right() const
{
    return _right;
}

// StereoCameraMatrix
StereoCameraMatrix::StereoCameraMatrix( const cv::Mat &left, const cv::Mat &right )
{
    set( left, right );
}

void StereoCameraMatrix::set( const cv::Mat &left, const cv::Mat &right )
{
    _left = left;
    _right = right;
}

const cv::Mat &StereoCameraMatrix::left() const
{
    return _left;
}

const cv::Mat &StereoCameraMatrix::right() const
{
    return _right;
}

// StereoDistorsionCoefficients
StereoDistorsionCoefficients::StereoDistorsionCoefficients( const cv::Mat &left, const cv::Mat &right )
{
    set( left, right );
}

void StereoDistorsionCoefficients::set( const cv::Mat &left, const cv::Mat &right )
{
    _left = left;
    _right = right;
}

const cv::Mat &StereoDistorsionCoefficients::left() const
{
    return _left;
}

const cv::Mat &StereoDistorsionCoefficients::right() const
{
    return _right;
}

// StereoCameraParameters
Parameters::Parameters()
{
    initialize();
}

void Parameters::initialize()
{
    _cornerExtractionCount = 1 << 10;
    _minimumTracksCount = 1 << 7;

    _minimumRecoverPointsCount = 1 << 5;
    _minimumInliersRatio = 0.7;

    _extractionDistance = 10.;

    _minimumStereoDisparity = 5.;

    auto tracker = std::make_shared< GPUFlowTracker >();

    setTracker( tracker );


}

void Parameters::setFrameSize( const cv::Size &value )
{
    _frameSize = value;
}

const cv::Size &Parameters::frameSize() const
{
    return _frameSize;
}

StereoRect Parameters::processRect() const
{
    return _procRect;
}

void Parameters::setProcessRect( const StereoRect &rect )
{
    _procRect = rect;
}

void Parameters::setCameraMatrix( const cv::Mat &left, const cv::Mat &right )
{
    _cameraMatrix.set( left, right );
}

void Parameters::setDistCoefficients( const cv::Mat &left, const cv::Mat &right )
{
    _distorsionCoefficients.set( left, right );
}

const StereoCameraMatrix &Parameters::cameraMatrix() const
{
    return _cameraMatrix;
}

const StereoDistorsionCoefficients &Parameters::distorsionCoefficients() const
{
    return _distorsionCoefficients;
}

void Parameters::setTracker( const std::shared_ptr< Tracker > &value )
{
    _tracker = value;
}

const std::shared_ptr< Tracker > &Parameters::tracker() const
{
    return _tracker;
}

void Parameters::setCornerExtractionCount( const size_t value )
{
    _cornerExtractionCount = value;
}

size_t Parameters::cornerExtractionCount() const
{
    return _cornerExtractionCount;
}

void Parameters::setMinimumTracksCount(  const size_t value )
{
    _minimumTracksCount = value;
}

size_t Parameters::minimumTracksCount() const
{
    return _minimumTracksCount;
}

void Parameters::setRightRotation( const cv::Mat &value )
{
    _rightRotation = value;
}

const cv::Mat &Parameters::rightRotation() const
{
    return _rightRotation;
}

void Parameters::setRightTranslation( const cv::Mat &value )
{
    _rightTranslation = value;
}

const cv::Mat &Parameters::rightTranslation() const
{
    return _rightTranslation;
}

double Parameters::maxReprojectionError() const
{
    return 3.;
}

size_t Parameters::minimumRecoverPointsCount() const
{
    return _minimumRecoverPointsCount;
}

double Parameters::minimumInliersRatio() const
{
    return _minimumInliersRatio;
}

double Parameters::extractionDistance() const
{
    return _extractionDistance;
}

double Parameters::minimumStereoDisparity() const
{
    return _minimumStereoDisparity;
}

double Parameters::pointsDrawScale() const
{
    return 1./500.;
}

double Parameters::textDrawScale() const
{
    return 1./70.;
}

}
