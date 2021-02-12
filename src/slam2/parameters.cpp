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

// StereoProjectionMatrix
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

    _maxReprojectionError = 3.;

    auto flowTracker = std::make_shared< GPUFlowTracker >();
    flowTracker->setRansacReprojectionThreshold( 3. );

    auto featureTracker = std::make_shared< SuperGlueTracker >();
    featureTracker->setRansacReprojectionThreshold( 3. );

    setFlowTracker( flowTracker );
    setFeatureTracker( featureTracker );

}

void Parameters::setCalibration( const StereoCalibrationDataShort &value )
{
    _calibration = value;
}

const StereoCalibrationDataShort &Parameters::calibration() const
{
    return _calibration;
}

StereoRect Parameters::processRect() const
{
    return _procRect;
}

void Parameters::setProcessRect( const StereoRect &rect )
{
    _procRect = rect;
}

StereoCameraMatrix Parameters::cameraMatrix() const
{
    return StereoCameraMatrix( _calibration.leftCameraResults().cameraMatrix(), _calibration.rightCameraResults().cameraMatrix() );
}

StereoDistorsionCoefficients Parameters::distorsionCoefficients() const
{
    return StereoDistorsionCoefficients( _calibration.leftCameraResults().distortionCoefficients(), _calibration.rightCameraResults().distortionCoefficients() );
}

void Parameters::setFlowTracker( const std::shared_ptr< FlowTracker > &value )
{
    _flowTracker = value;
}

const std::shared_ptr< FlowTracker > &Parameters::flowTracker() const
{
    return _flowTracker;
}

void Parameters::setFeatureTracker( const std::shared_ptr< FeatureTracker > &value )
{
    _featureTracker = value;
}

const std::shared_ptr< FeatureTracker > &Parameters::featureTracker() const
{
    return _featureTracker;
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

cv::Mat Parameters::rightRotation() const
{
    return _calibration.rotationMatrix();
}

cv::Mat Parameters::rightTranslation() const
{
    return _calibration.translationVector();
}

double Parameters::maxReprojectionError() const
{
    return _maxReprojectionError;
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
