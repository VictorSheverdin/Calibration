#include "src/common/precompiled.h"

#include "frame.h"

#include "system.h"
#include "map.h"

#include "parameters.h"

#include "framepoint.h"

#include "tracker.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

namespace slam2 {

// Frame
Frame::Frame( const StereoFramePtr &parent )
    : Parent_Shared_Ptr< StereoFrame >( parent )
{
}

std::shared_ptr< StereoFrame > Frame::parentStereoFrame() const
{
    return parentPointer();
}

std::shared_ptr< Map > Frame::parentMap() const
{
    return parentStereoFrame()->parentMap();
}

std::shared_ptr< System > Frame::parentSystem() const
{
    return parentMap()->parentSystem();
}

// FinalFrame
FinalFrame::FinalFrame( const StereoFramePtr &parent )
    : Frame( parent )
{
    initialize();
}

void FinalFrame::initialize()
{
    setCameraMatrix( cv::Mat::eye( 3, 3, CV_64F ) );
}

FinalFrame::ObjectPtr FinalFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinalFrame >( Frame::shared_from_this() );
}

FinalFrame::ObjectConstPtr FinalFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinalFrame >( Frame::shared_from_this() );
}

void FinalFrame::setTime( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    _time = time;
}

const std::chrono::time_point< std::chrono::system_clock > &FinalFrame::time() const
{
    return _time;
}

void FinalFrame::setCameraMatrix( const cv::Mat &value )
{
    _cameraMatrix = value;
}

const cv::Mat &FinalFrame::cameraMatrix() const
{
    return _cameraMatrix;
}

// ProcFrame
ProcFrame::ProcFrame( const StereoFramePtr &parent )
    : FinalFrame( parent )
{
}

ProcFrame::ObjectPtr ProcFrame::create( const StereoFramePtr &parent )
{
    return ObjectPtr( new ProcFrame( parent ) );
}

void ProcFrame::load( const StampedImage &image )
{
    setTime( image.time() );

    _image = image;
}

const cv::Point2f &ProcFrame::cornerPoint( const size_t index ) const
{
    return _cornerPoints[ index ];
}

const cv::Point2f &ProcFrame::undistortedCornerPoint( const size_t index ) const
{
    return _undistCornerPoints[ index ];
}

const cv::Point2f &ProcFrame::featurePoint( const size_t index ) const
{
    return _featurePoints[ index ].pt;
}

const cv::Point2f &ProcFrame::undistortedFeaturePoint( const size_t index ) const
{
    return _undistFeaturePoints[ index ];
}

ProcFrame::ObjectPtr ProcFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< ProcFrame >( Frame::shared_from_this() );
}

ProcFrame::ObjectConstPtr ProcFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const ProcFrame >( Frame::shared_from_this() );
}

void ProcFrame::setDistorsionCoefficients( const cv::Mat &value )
{
    _distCoefficients = value;
}

const cv::Mat &ProcFrame::distorsionCoefficients() const
{
    return _distCoefficients;
}

const StampedImage &ProcFrame::image() const
{
    return _image;
}

cv::Mat ProcFrame::mask() const
{
    // TODO:
    //cv::Mat ret( _image.rows, _image.cols, CV_8U, cv::Scalar( 1 ) );

    //auto system = parentSystem();

    //auto tracker = system->flowTracker();

    //auto distance = tracker->extractionDistance();

    //for ( auto &i : _cornerPoints )
        //cv::rectangle( ret, cv::Rect( i.x - distance, i.y - distance, distance * 2, distance * 2 ), cv::Scalar( 0 ) );

    //return ret;

    return cv::Mat();
}

FlowPointPtr ProcFrame::createFlowPoint( const size_t index )
{
    auto point = FlowPoint::create( shared_from_this(), index );

    _corners.push_back( point );

    return point;
}

FeaturePointPtr ProcFrame::createFeaturePoint( const size_t index )
{
    auto point = FeaturePoint::create( shared_from_this(), index );

    _features.push_back( point );

    return point;
}

size_t ProcFrame::addCornerPoint( const cv::Point2f &point )
{
    auto ret = _cornerPoints.size();

    std::vector< cv::Point2f > points, undistortedPoints;
    points.push_back( point );

    undistortPoints( points, &undistortedPoints );

    _cornerPoints.push_back( point );
    _undistCornerPoints.push_back( undistortedPoints.front() );

    return ret;
}

size_t ProcFrame::addCornerPoints( const std::vector< cv::Point2f > &points )
{
    auto ret = _cornerPoints.size();

    std::vector< cv::Point2f > undistortedPoints;

    undistortPoints( points, &undistortedPoints );

    _cornerPoints.insert( _cornerPoints.end(), points.begin(), points.end() );
    _undistCornerPoints.insert( _undistCornerPoints.end(), undistortedPoints.begin(), undistortedPoints.end() );

    return ret;
}

void ProcFrame::setFeaturePoints( const std::vector< cv::KeyPoint > &value )
{
    std::vector< cv::Point2f > points;

    for ( auto &i : value )
        points.push_back( i.pt );

    undistortPoints( points, &_undistFeaturePoints );

    _featurePoints = value;
}

void ProcFrame::undistortPoints( const std::vector< cv::Point2f > &sourcePoints, std::vector< cv::Point2f > *undistortedPoints ) const
{
    cv::undistortPoints( sourcePoints, *undistortedPoints, _cameraMatrix, _distCoefficients );

    for ( auto &i : *undistortedPoints ) {
        cv::Mat pt( 3, 1, CV_64F );
        pt.at< double >( 0, 0 ) = i.x;
        pt.at< double >( 1, 0 ) = i.y;
        pt.at< double >( 2, 0 ) = 1;

        cv::Mat mul = _cameraMatrix * pt;

        auto w = mul.at< double >( 2, 0 );

        if ( std::abs( w ) > FLOAT_EPS ) {
            i.x = mul.at< double >( 0, 0 ) / w;
            i.y = mul.at< double >( 1, 0 ) / w;
        }
        else {
            i.x = 0.;
            i.y = 0.;
        }

    }

}


const std::vector< cv::KeyPoint > &ProcFrame::featurePoints() const
{
    return _featurePoints;
}

void ProcFrame::setImagePyramid( const std::vector< cv::Mat > &value )
{
    _imagePyramid = value;
}

const std::vector< cv::Mat > &ProcFrame::imagePyramid() const
{
    return _imagePyramid;
}

const std::vector< cv::Point2f > &ProcFrame::cornerPoints() const
{
    return _cornerPoints;
}

void ProcFrame::setDescriptors( const cv::Mat &value )
{
    _descriptors = value;
}

const cv::Mat &ProcFrame::descriptors() const
{
    return _descriptors;
}

size_t ProcFrame::extractionCornersCount() const
{
    // TODO:
    auto system = parentSystem();

    return system->parameters().cornerExtractionCount();
}

// StereoFrame
StereoFrame::StereoFrame( const MapPtr &parent )
    : Parent_Shared_Ptr< Map >( parent )
{
    initialize();
}

void StereoFrame::initialize()
{
    setRotation( cv::Mat::eye( 3, 3, CV_64F ) );
    setTranslation( cv::Mat::zeros( 3, 1, CV_64F ) );
}

void StereoFrame::set( const FramePtr &leftFrame, const FramePtr &rightFrame )
{
    _leftFrame = leftFrame;
    _rightFrame = rightFrame;
}

std::shared_ptr< Map > StereoFrame::parentMap() const
{
    return parentPointer();
}

std::shared_ptr< System > StereoFrame::parentSystem() const
{
    return parentMap()->parentSystem();
}

void StereoFrame::setRotation( const cv::Mat &value )
{
    _rotation = value;
}

const cv::Mat &StereoFrame::rotation() const
{
    return _rotation;
}

void StereoFrame::setTranslation( const cv::Mat &value )
{
    _translation = value;
}

const cv::Mat &StereoFrame::translation() const
{
    return _translation;
}

void StereoFrame::setRightRotation( const cv::Mat &value )
{
    _rightRotation = value;
}

const cv::Mat &StereoFrame::rightRotation() const
{
    return _rightRotation;
}

void StereoFrame::setRightTranslation( const cv::Mat &value )
{
    _rightTranslation = value;
}

const cv::Mat &StereoFrame::rightTranslation() const
{
    return _rightTranslation;
}

// FinalStereoFrame
FinalStereoFrame::FinalStereoFrame( const MapPtr &parent )
    : StereoFrame( parent )
{
}

FinalStereoFrame::ObjectPtr FinalStereoFrame::create( const MapPtr &parent )
{
    return ObjectPtr( new FinalStereoFrame( parent ) );
}

FinalFramePtr FinalStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FinalFrame >( _leftFrame );
}

FinalFramePtr FinalStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FinalFrame >( _rightFrame );
}

void FinalStereoFrame::setCameraMatrices( const StereoCameraMatrix &value )
{
    leftFrame()->setCameraMatrix( value.left() );
    rightFrame()->setCameraMatrix( value.right() );
}

ProjectionMatrix FinalStereoFrame::leftProjectionMatrix() const
{
    ProjectionMatrix ret;

    ret.setCameraMatrix( leftFrame()->cameraMatrix() );
    ret.setRotation( rotation() );
    ret.setTranslation( translation() );

    return ret;
}

ProjectionMatrix FinalStereoFrame::rightProjectionMatrix() const
{
    ProjectionMatrix ret;

    ret.setCameraMatrix( rightFrame()->cameraMatrix() );
    ret.setRotation( rotation() * rightRotation() );
    ret.setTranslation( translation() + rightTranslation() );

    return ret;
}

FinalStereoFrame::ObjectPtr FinalStereoFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinalStereoFrame >( StereoFrame::shared_from_this() );
}

FinalStereoFrame::ObjectConstPtr FinalStereoFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinalStereoFrame >( StereoFrame::shared_from_this() );
}

// ProcStereoFrame
ProcStereoFrame::ProcStereoFrame( const MapPtr &parent )
    : FinalStereoFrame( parent )
{
}

ProcStereoFrame::ObjectPtr ProcStereoFrame::create( const MapPtr &parent )
{
    auto frame = ObjectPtr( new ProcStereoFrame( parent ) );

    frame->set( ProcFrame::create( frame ), ProcFrame::create( frame ) );

    return frame;
}

void ProcStereoFrame::load( const StampedStereoImage &image )
{
    leftFrame()->load( image.leftImage() );
    rightFrame()->load( image.rightImage() );
}

void ProcStereoFrame::prepareFrame()
{
    auto system = parentSystem();

    auto tracker = system->tracker();

    tracker->prepareFrame( this );
}

void ProcStereoFrame::extract()
{
    auto system = parentSystem();

    auto tracker = system->tracker();

    tracker->extractFeatures( this );
}

size_t ProcStereoFrame::triangulatePoints()
{
    size_t ret = 0;

    auto system = parentSystem();

    auto maxReprojectionError = system->parameters().maxReprojectionError();

    cv::Mat homogeneousPoints3d;

    cv::Mat_< float > leftPoints( 2, _points.size() ), rightPoints( 2, _points.size() );

    for ( size_t i = 0; i < _points.size(); ++i ) {
        auto leftPoint = _points[i]->leftPoint()->undistortedPoint();
        auto rightPoint = _points[i]->rightPoint()->undistortedPoint();
        leftPoints.row( 0 ).col( i ) = leftPoint.x;
        leftPoints.row( 1 ).col( i ) = leftPoint.y;
        rightPoints.row( 0 ).col( i ) = rightPoint.x;
        rightPoints.row( 1 ).col( i ) = rightPoint.y;
    }

    auto leftProjectionMatrix = this->leftProjectionMatrix().projectionMatrix();
    auto rightProjectionMatrix = this->rightProjectionMatrix().projectionMatrix();

    cv::triangulatePoints( leftProjectionMatrix, rightProjectionMatrix, leftPoints, rightPoints, homogeneousPoints3d );

    for ( size_t i = 0; i < _points.size(); ++i ) {

        auto w = homogeneousPoints3d.at< float >( 3, i );

        if ( std::abs( w ) > FLOAT_EPS ) {

            auto x = homogeneousPoints3d.at< float >( 0, i ) / w;
            auto y = homogeneousPoints3d.at< float >( 1, i ) / w;
            auto z = homogeneousPoints3d.at< float >( 2, i ) / w;

            auto pt = cv::Point3f( x, y, z );

            cv::Mat pt4d( 4, 1, CV_64F );
            homogeneousPoints3d.col( i ).convertTo( pt4d, CV_64F );

            cv::Mat leftReprojMat = leftProjectionMatrix * pt4d;
            cv::Mat rightReprojMat = rightProjectionMatrix * pt4d;

            auto leftW = leftReprojMat.at< double >( 2, 0 );
            auto rightW = rightReprojMat.at< double >( 2, 0 );

            if ( std::abs( leftW ) > DOUBLE_EPS && std::abs( rightW ) > DOUBLE_EPS && leftW / w > 0 && rightW / w > 0 ) {

                cv::Point2f leftReprojPt( leftReprojMat.at< double >( 0, 0 ) / leftW,
                                          leftReprojMat.at< double >( 1, 0 ) / leftW );

                cv::Point2f rightReprojPt( rightReprojMat.at< double >( 0, 0 ) / rightW,
                                          rightReprojMat.at< double >( 1, 0 ) / rightW );

                auto leftNorm = cv::norm( leftReprojPt - _points[i]->leftPoint()->undistortedPoint() );
                auto rightNorm = cv::norm( rightReprojPt - _points[i]->rightPoint()->undistortedPoint() );

                if ( leftNorm < maxReprojectionError && rightNorm < maxReprojectionError ) {

                    ++ret;

                    _points[i]->setPoint3d( pt );

                }

            }

        }

    }

    return ret;

}

ProcFramePtr ProcStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< ProcFrame >( _leftFrame );
}

ProcFramePtr ProcStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcFrame >( _rightFrame );
}

void ProcStereoFrame::setDistorsionCoefficients( const StereoDistorsionCoefficients &value )
{
    leftFrame()->setDistorsionCoefficients( value.left() );
    rightFrame()->setDistorsionCoefficients( value.right() );
}

ProcStereoFrame::ObjectPtr ProcStereoFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< ProcStereoFrame >( StereoFrame::shared_from_this() );
}

ProcStereoFrame::ObjectConstPtr ProcStereoFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const ProcStereoFrame >( StereoFrame::shared_from_this() );
}

FlowStereoPointPtr ProcStereoFrame::createFlowPoint( const size_t leftIndex , const size_t rightIndex )
{
    auto leftPoint = leftFrame()->createFlowPoint( leftIndex );
    auto rightPoint = rightFrame()->createFlowPoint( rightIndex );

    auto stereoPoint = FlowStereoPoint::create( leftPoint, rightPoint );

    _points.push_back( stereoPoint );

    return stereoPoint;
}

FeatureStereoPointPtr ProcStereoFrame::createFeaturePoint( const size_t leftIndex, const size_t rightIndex )
{
    auto leftPoint = leftFrame()->createFeaturePoint( leftIndex );
    auto rightPoint = rightFrame()->createFeaturePoint( rightIndex );

    auto stereoPoint = FeatureStereoPoint::create( leftPoint, rightPoint );

    _points.push_back( stereoPoint );

    return stereoPoint;
}

CvImage ProcStereoFrame::drawPoints() const
{
    auto system = parentSystem();

    CvImage leftImage, rightImage;

    leftFrame()->image().copyTo( leftImage );
    rightFrame()->image().copyTo( rightImage );

    auto leftCorners = leftFrame()->cornerPoints();
    auto leftKeypoints = leftFrame()->featurePoints();
    auto rightCorners = rightFrame()->cornerPoints();
    auto rightKeypoints = rightFrame()->featurePoints();

    double radius;

    auto pointDrawScale = system->parameters().pointsDrawScale();

    radius = std::min( leftImage.width(), leftImage.height() ) * pointDrawScale;

    drawFeaturePoints( &leftImage, leftCorners, radius, cv::Scalar( 0, 0, 255, 255 ) );
    drawFeaturePoints( &leftImage, leftKeypoints, radius, cv::Scalar( 0, 255, 0, 255 ) );

    radius = std::min( rightImage.width(), rightImage.height() ) * pointDrawScale;

    drawFeaturePoints( &rightImage, rightKeypoints, radius, cv::Scalar( 0, 255, 0, 255 ) );
    drawFeaturePoints( &rightImage, rightCorners, radius, cv::Scalar( 0, 0, 255, 255 ) );

    return makeStraightPreview( leftImage, rightImage );
}

CvImage ProcStereoFrame::drawTracks() const
{
    auto stackedImage = makeStraightPreview( leftFrame()->image(), rightFrame()->image() );

    return stackedImage;
}

CvImage ProcStereoFrame::drawStereo() const
{
    auto stackedImage = makeStraightPreview( leftFrame()->image(), rightFrame()->image() );

    auto system = parentSystem();

    auto pointDrawScale = system->parameters().pointsDrawScale();

    auto radius = std::min( stackedImage.width(), stackedImage.height() ) * pointDrawScale;

    for ( auto &i :_points ) {

        auto left = i->leftPoint()->point2d();
        auto right = i->rightPoint()->point2d();

        auto offset = cv::Point2f( leftFrame()->image().width(), 0 );

        drawLine( &stackedImage, left, right + offset, cv::Scalar( 0, 255, 0, 255 ) );
        drawFeaturePoint( &stackedImage, left, radius, cv::Scalar( 0, 0, 255, 255 ) );
        drawFeaturePoint( &stackedImage, right + offset, radius, cv::Scalar( 0, 0, 255, 255 ) );

    }

    return stackedImage;
}

std::vector< ColorPoint3d > ProcStereoFrame::sparseCloud() const
{
    std::vector< ColorPoint3d > ret;

    for ( auto &i : _points )
        if ( i->isPoint3dExist() )
            ret.push_back( ColorPoint3d( i->point3d(), i->color() ) );

    return ret;
}

void ProcStereoFrame::setImagePyramid( const std::vector< cv::Mat > &leftPyramid, const std::vector< cv::Mat > &rightPyramid )
{
    leftFrame()->setImagePyramid( leftPyramid );
    rightFrame()->setImagePyramid( rightPyramid );
}

void ProcStereoFrame::setFeaturePoints( const std::vector< cv::KeyPoint > &left, const std::vector< cv::KeyPoint > &right )
{
    leftFrame()->setFeaturePoints( left );
    rightFrame()->setFeaturePoints( right );
}

void ProcStereoFrame::setDescriptors( const cv::Mat &left, const cv::Mat &right )
{
    leftFrame()->setDescriptors( left );
    rightFrame()->setDescriptors( right );
}

}
