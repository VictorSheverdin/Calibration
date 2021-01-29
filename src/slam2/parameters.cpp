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
    _cornerExtractionCount = 5e2;

    setTracker( std::make_shared< SuperGlueTracker >() );
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

StereoRect Parameters::processRect() const
{
    return _procRect;
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
    return 2.;
}

double Parameters::pointsDrawScale() const
{
    return 1./500.;
}

}
