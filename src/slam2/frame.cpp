#include "src/common/precompiled.h"

#include "frame.h"

#include "system.h"
#include "map.h"

#include "framepoint.h"
#include "mappoint.h"

#include "track.h"

#include "tracker.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

namespace slam2 {

// Frame
Frame::Frame( const StereoFramePtr &parent )
    : Parent_Weak_Ptr< StereoFrame >( parent )
{
}

void Frame::setTime( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    _time = time;
}

const std::chrono::time_point< std::chrono::system_clock > &Frame::time() const
{
    return _time;
}

void Frame::setCameraMatrix( const cv::Mat &value )
{
    _cameraMatrix = value;
}

const cv::Mat &Frame::cameraMatrix() const
{
    return _cameraMatrix;
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

TrackPtr Frame::createTrack()
{
    auto track = Track::create();

    _tracks.push_back( track );

    return track;
}

void Frame::setTracks( const std::vector< TrackPtr > &value )
{
    _tracks = value;
}

const std::vector< TrackPtr > &Frame::tracks() const
{
    return _tracks;
}

// FinalFrame
FinalFrame::FinalFrame( const FinalStereoFramePtr &parent )
    : Frame( parent )
{
    initialize();
}

void FinalFrame::initialize()
{
    setCameraMatrix( cv::Mat::eye( 3, 3, CV_64F ) );
}

FinalFrame::ObjectPtr FinalFrame::create( const FinalStereoFramePtr &parent )
{
    return ObjectPtr( new FinalFrame( parent ) );
}

FinalFrame::ObjectPtr FinalFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinalFrame >( Frame::shared_from_this() );
}

FinalFrame::ObjectConstPtr FinalFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinalFrame >( Frame::shared_from_this() );
}

void FinalFrame::addPoint( const FinalPointPtr &point )
{
    _points.push_back( point );
}

// ProcFrame
const cv::Scalar ProcFrame::_recoverTrackColor( 0, 255, 0, 255 );
const cv::Scalar ProcFrame::_otherTrackColor( 255, 0, 0, 255 );

ProcFrame::ProcFrame( const ProcStereoFramePtr &parent )
    : Frame( parent )
{
}

ProcFrame::ObjectPtr ProcFrame::create( const ProcStereoFramePtr &parent )
{
    return ObjectPtr( new ProcFrame( parent ) );
}

void ProcFrame::load( const StampedImage &image )
{
    setTime( image.time() );

    _image = image;

    _imagePyramid.clear();

    _cornerPoints.clear();
    _undistCornerPoints.clear();

    _flowPoints.clear();

    _keyPoints.clear();
    _undistKeyPoints.clear();
    _descriptors.release();

    _featurePoints.clear();
}

const cv::Point2f &ProcFrame::cornerPoint( const size_t index ) const
{
    return _cornerPoints[ index ];
}

const cv::Point2f &ProcFrame::undistortedCornerPoint( const size_t index ) const
{
    return _undistCornerPoints[ index ];
}

const cv::KeyPoint &ProcFrame::keyPoint( const size_t index ) const
{
    return _keyPoints[ index ];
}

const cv::KeyPoint &ProcFrame::undistortedKeyPoint( const size_t index ) const
{
    return _undistKeyPoints[ index ];
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

void ProcFrame::setMask( const cv::Mat &mask )
{
    _mask = mask;
}

cv::Mat ProcFrame::mask() const
{
    cv::Mat ret( _image.size(), CV_8U, 1 );

    if ( !_mask.empty() )
        _mask.copyTo( ret );

    auto system = parentSystem();

    auto distance = system->parameters().extractionDistance();

    for ( auto &i : _cornerPoints )
        cv::rectangle( ret, cv::Rect( i.x - distance, i.y - distance, distance * 2, distance * 2 ), 0, cv::FILLED );

    for ( auto &i : _keyPoints ) {
        auto pt = i.pt;
        cv::rectangle( ret, cv::Rect( pt.x - distance, pt.y - distance, distance * 2, distance * 2 ), 0, cv::FILLED );
    }

    return ret;

}

FlowPointPtr ProcFrame::createFlowPoint( const size_t index )
{
    auto it = _flowPoints.find( index );

    if ( it != _flowPoints.end() )
        return it->second;

    auto point = FlowPoint::create( shared_from_this(), index );

    _flowPoints[ index ] = point;

    return point;
}

FeaturePointPtr ProcFrame::createFeaturePoint( const size_t index )
{
    auto it = _featurePoints.find( index );

    if ( it != _featurePoints.end() )
        return it->second;

    auto point = FeaturePoint::create( shared_from_this(), index );

    _featurePoints[ index ] = point;

    return point;
}

FlowPointPtr ProcFrame::flowPoint( const size_t index ) const
{
    auto it = _flowPoints.find( index );

    if ( it != _flowPoints.end() )
        return it->second;
    else
        return FlowPointPtr();

}

FeaturePointPtr ProcFrame::featurePoint( const size_t index ) const
{
    auto it = _featurePoints.find( index );

    if ( it != _featurePoints.end() )
        return it->second;
    else
        return FeaturePointPtr();

}

const std::map< size_t, FlowPointPtr > &ProcFrame::flowPoints() const
{
    return _flowPoints;
}

std::vector< FlowPointPtr > ProcFrame::flowPointsVector()
{
    std::vector< FlowPointPtr > ret;

    ret.reserve( _flowPoints.size() );

    for ( auto &i : _flowPoints )
        ret.push_back( i.second );

    return ret;
}

const std::map< size_t, FeaturePointPtr > &ProcFrame::featurePoints() const
{
    return _featurePoints;
}

std::vector< FeaturePointPtr > ProcFrame::featurePointsVector()
{
    std::vector< FeaturePointPtr > ret;

    ret.reserve( _featurePoints.size() );

    for ( auto &i : _featurePoints )
        ret.push_back( i.second );

    return ret;
}

CvImage ProcFrame::drawPoints() const
{
    CvImage ret;

    auto system = parentSystem();

    image().copyTo( ret );

    double radius;

    auto pointDrawScale = system->parameters().pointsDrawScale();

    radius = std::min( ret.width(), ret.height() ) * pointDrawScale;

    drawFeaturePoints( &ret, _cornerPoints, radius, cv::Scalar( 0, 0, 255, 100 ) );
    drawFeaturePoints( &ret, _keyPoints, radius, cv::Scalar( 0, 255, 0, 100 ) );

    drawLabel( &ret, "Points count: " + std::to_string( _cornerPoints.size() + _keyPoints.size() ), std::min( ret.height(), ret.width() ) * system->parameters().textDrawScale() );

    return ret;
}

CvImage ProcFrame::drawTracks() const
{
    CvImage ret;

    size_t recoverTracksCount = 0;
    size_t allCount = 0;

    auto system = parentSystem();

    image().copyTo( ret );

    double radius;

    auto pointDrawScale = system->parameters().pointsDrawScale();

    radius = std::min( ret.width(), ret.height() ) * pointDrawScale;

    std::vector< cv::Point2f > points;

    for ( auto &i : _flowPoints ) {

        points.push_back( i.second->point2d() );

        auto track = i.second->parentTrack();

        if ( track ) {

            cv::Scalar color;

            ++allCount;

            if ( track->mapPoint() ) {
                ++recoverTracksCount;
                color = _recoverTrackColor;
            }
            else
                color = _otherTrackColor;

            auto points = track->pointsVector();

            Point2Ptr prev;

            for ( auto &j : points ) {
                if ( !prev )
                    prev = j;
                else {
                    drawLine( &ret, prev->point2d(), j->point2d(), 2, color );
                    prev = j;
                }

            }

        }

    }

    for ( auto &i : _featurePoints ) {

        points.push_back( i.second->point2d() );

        auto track = i.second->parentTrack();

        if ( track ) {

            cv::Scalar color;

            ++allCount;

            if ( track->mapPoint() ) {
                ++recoverTracksCount;
                color = _recoverTrackColor;
            }
            else
                color = _otherTrackColor;

            auto points = track->pointsVector();

            Point2Ptr prev;

            for ( auto &j : points ) {
                if ( !prev )
                    prev = j;
                else {
                    drawLine( &ret, prev->point2d(), j->point2d(), 2, color );
                    prev = j;
                }

            }

        }

    }

    drawFeaturePoints( &ret, points, radius, cv::Scalar( 0, 0, 255, 255 ) );

    drawLabel( &ret, "Recover tracks count: " + std::to_string( recoverTracksCount ) + ", all tracks count: " + std::to_string( allCount ),
                    std::min( ret.height(), ret.width() ) * system->parameters().textDrawScale() );

    return ret;
}

std::vector< ProcPointPtr > ProcFrame::recoverPoints() const
{
    std::vector< ProcPointPtr > ret;

    for ( auto &i : _flowPoints )
        if ( i.second->parentTrack() && i.second->parentTrack()->mapPoint() )
            ret.push_back( i.second );

    for ( auto &i : _featurePoints )
        if ( i.second->parentTrack() && i.second->parentTrack()->mapPoint() )
            ret.push_back( i.second );

    return ret;

}

size_t ProcFrame::recoverPointsCount() const
{
    return recoverPoints().size();
}

cv::Scalar ProcFrame::color( const cv::Point2f &point ) const
{
    return _image.at< cv::Vec3b >( point );
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

    if ( !points.empty() ) {

        std::vector< cv::Point2f > undistortedPoints;

        undistortPoints( points, &undistortedPoints );

        _cornerPoints.insert( _cornerPoints.end(), points.begin(), points.end() );
        _undistCornerPoints.insert( _undistCornerPoints.end(), undistortedPoints.begin(), undistortedPoints.end() );

    }

    return ret;
}

void ProcFrame::setKeyPoints( const std::vector< cv::KeyPoint > &value )
{
    _keyPoints = value;

    std::vector< cv::Point2f > points;

    for ( auto &i : value )
        points.push_back( i.pt );

    std::vector< cv::Point2f > undistPoints;

    undistortPoints( points, &undistPoints );

    _undistKeyPoints.clear();

    for ( size_t i = 0; i < value.size(); ++i ) {
        cv::KeyPoint keyPoint = _keyPoints[ i ];
        keyPoint.pt = undistPoints[ i ];
        _undistKeyPoints.push_back( keyPoint );
    }

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

        if ( std::abs( w ) > DOUBLE_EPS ) {
            i.x = mul.at< double >( 0, 0 ) / w;
            i.y = mul.at< double >( 1, 0 ) / w;
        }
        else {
            i.x = 0.;
            i.y = 0.;
        }

    }

}


const std::vector< cv::KeyPoint > &ProcFrame::keyPoints() const
{
    return _keyPoints;
}

const std::vector< cv::KeyPoint > &ProcFrame::undistortedKeyPoints() const
{
    return _undistKeyPoints;
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

const std::vector< cv::Point2f > &ProcFrame::undistortedCornerPoints() const
{
    return _undistCornerPoints;
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

// DoubleFrame
DoubleFrame::DoubleFrame( const MapPtr &parent )
    : Parent_Weak_Ptr< Map >( parent )
{
}

void DoubleFrame::set( const FramePtr &frame1, const FramePtr &frame2 )
{
    _frame1 = frame1;
    _frame2 = frame2;
}

const FramePtr &DoubleFrame::frame1()
{
    return _frame1;
}

const FramePtr &DoubleFrame::frame2()
{
    return _frame2;
}

FrameConstPtr DoubleFrame::frame1() const
{
    return _frame1;
}

FrameConstPtr DoubleFrame::frame2() const
{
    return _frame2;
}

std::shared_ptr< Map > DoubleFrame::parentMap() const
{
    return parentPointer();
}

std::shared_ptr< System > DoubleFrame::parentSystem() const
{
    return parentMap()->parentSystem();
}

// StereoFrame
StereoFrame::StereoFrame( const MapPtr &parent )
    : DoubleFrame( parent )
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
    DoubleFrame::set( leftFrame, rightFrame );
}

const FramePtr &StereoFrame::leftFrame()
{
    return frame1();
}

const FramePtr &StereoFrame::rightFrame()
{
    return frame2();
}

FrameConstPtr StereoFrame::leftFrame() const
{
    return frame1();
}

FrameConstPtr StereoFrame::rightFrame() const
{
    return frame2();
}

void StereoFrame::setCameraMatrices( const StereoCameraMatrix &value )
{
    leftFrame()->setCameraMatrix( value.left() );
    rightFrame()->setCameraMatrix( value.right() );
}

cv::Mat StereoFrame::leftProjectionMatrix() const
{
    return calcProjectionMatrix( leftFrame()->cameraMatrix(), rotation(), translation() );
}

cv::Mat StereoFrame::rightProjectionMatrix() const
{
    cv::Mat m1 = cv::Mat::eye( 4, 4, CV_64F );
    rightRotation().copyTo( m1.rowRange( 0, 3 ).colRange( 0, 3 ) );
    rightTranslation().copyTo( m1.rowRange( 0, 3 ).col( 3 ) );

    cv::Mat m2 = cv::Mat::eye( 4, 4, CV_64F );
    rotation().copyTo( m2.rowRange( 0, 3 ).colRange( 0, 3 ) );
    translation().copyTo( m2.rowRange( 0, 3 ).col( 3 ) );

    cv::Mat mul = m1 * m2;

    return calcProjectionMatrix( rightFrame()->cameraMatrix(), mul.rowRange( 0, 3 ).colRange( 0, 3 ), mul.rowRange( 0, 3 ).col( 3 ) );
}

StereoProjectionMatrix StereoFrame::projectionMatrix() const
{
    return StereoProjectionMatrix( ProjectionMatrix( leftProjectionMatrix() ), ProjectionMatrix( rightProjectionMatrix() ) );
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

std::vector< ColorPoint3d > StereoFrame::sparseCloud() const
{
    std::vector< ColorPoint3d > ret;

    for ( auto &i : leftFrame()->tracks() ) {
        auto mapPoint = i->mapPoint();
        if ( mapPoint )
            ret.push_back( mapPoint->point() );
    }

    return ret;
}

// FinalStereoFrame
FinalStereoFrame::FinalStereoFrame( const MapPtr &parent )
    : StereoFrame( parent )
{
}

FinalStereoFrame::ObjectPtr FinalStereoFrame::create( const MapPtr &parent )
{
    auto frame = ObjectPtr( new FinalStereoFrame( parent ) );

    frame->set( FinalFrame::create( frame ), FinalFrame::create( frame ) );

    return frame;
}

FinalFramePtr FinalStereoFrame::leftFrame()
{
    return std::dynamic_pointer_cast< FinalFrame >( StereoFrame::leftFrame() );
}

FinalFramePtr FinalStereoFrame::rightFrame()
{
    return std::dynamic_pointer_cast< FinalFrame >( StereoFrame::rightFrame() );
}

FinalFrameConstPtr FinalStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< const FinalFrame >( StereoFrame::leftFrame() );
}

FinalFrameConstPtr FinalStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< const FinalFrame >( StereoFrame::rightFrame() );
}

FinalStereoFrame::ObjectPtr FinalStereoFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinalStereoFrame >( StereoFrame::shared_from_this() );
}

FinalStereoFrame::ObjectConstPtr FinalStereoFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinalStereoFrame >( StereoFrame::shared_from_this() );
}

void FinalStereoFrame::replace( ProcStereoFramePtr source )
{
    auto sourceLeftFrame = source->leftFrame();
    auto sourceRightFrame = source->rightFrame();

    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    leftFrame->setTime( sourceLeftFrame->time() );
    leftFrame->setCameraMatrix( sourceLeftFrame->cameraMatrix() );

    rightFrame->setTime( sourceRightFrame->time() );
    rightFrame->setCameraMatrix( sourceRightFrame->cameraMatrix() );

    setRotation( source->rotation() );
    setTranslation( source->translation() );

    setRightRotation( source->rightRotation() );
    setRightTranslation( source->rightTranslation() );

    leftFrame->setTracks( sourceLeftFrame->tracks() );
    rightFrame->setTracks( sourceRightFrame->tracks() );

    auto &stereoPoints = source->stereoPoints();

    for ( auto &i : stereoPoints ) {
        auto leftSourcePoint = i->leftPoint();
        auto rightSourcePoint = i->rightPoint();

        auto leftPoint = FinalPoint::create( leftFrame );
        leftPoint->setPoint( leftSourcePoint->undistortedPoint() );
        leftPoint->setColor( leftSourcePoint->color() );
        auto leftParentTrack = leftSourcePoint->parentTrack();
        if ( leftParentTrack )
            leftParentTrack->setPoint( leftSourcePoint->trackIndex(), leftPoint );

        leftFrame->addPoint( leftPoint );

        auto rightPoint = FinalPoint::create( rightFrame );
        rightPoint->setPoint( rightSourcePoint->undistortedPoint() );
        rightPoint->setColor( rightSourcePoint->color() );
        auto rightParentTrack = rightSourcePoint->parentTrack();
        if ( rightParentTrack )
            rightParentTrack->setPoint( rightSourcePoint->trackIndex(), rightPoint );

        rightFrame->addPoint( rightPoint );

        auto stereoPoint = FinalStereoPoint::create();

        stereoPoint->set( leftPoint, rightPoint );

        _points.push_back( stereoPoint );
    }

    std::vector< ProcPointPtr > leftProcPoints;
    auto leftFlowPoints = sourceLeftFrame->flowPointsVector();
    auto leftFeaturePoints = sourceLeftFrame->featurePointsVector();
    leftProcPoints.reserve( leftFlowPoints.size() + leftFeaturePoints.size() );
    leftProcPoints.insert( leftProcPoints.end(), leftFlowPoints.begin(), leftFlowPoints.end() );
    leftProcPoints.insert( leftProcPoints.end(), leftFeaturePoints.begin(), leftFeaturePoints.end() );

    for ( auto &i : leftProcPoints ) {

        auto parentTrack = i->parentTrack();

        if ( !i->stereoPoint() && parentTrack ) {

            auto procPoint = FinalPoint::create( leftFrame );
            procPoint->setPoint( i->undistortedPoint() );
            procPoint->setColor( i->color() );
            parentTrack->setPoint( i->trackIndex(), procPoint );

            leftFrame->addPoint( procPoint );

        }

    }

    std::vector< ProcPointPtr > rightProcPoints;
    auto rightFlowPoints = sourceRightFrame->flowPointsVector();
    auto rightFeaturePoints = sourceRightFrame->flowPointsVector();
    rightProcPoints.reserve( rightFlowPoints.size() + rightFeaturePoints.size() );
    rightProcPoints.insert( rightProcPoints.end(), rightFlowPoints.begin(), rightFlowPoints.end() );
    rightProcPoints.insert( rightProcPoints.end(), rightFeaturePoints.begin(), rightFeaturePoints.end() );

    for ( auto &i : rightProcPoints ) {

        auto parentTrack = i->parentTrack();

        if ( !i->stereoPoint() && parentTrack ) {

            auto procPoint = FinalPoint::create( rightFrame );
            procPoint->setPoint( i->undistortedPoint() );
            procPoint->setColor( i->color() );
            parentTrack->setPoint( i->trackIndex(), procPoint );

            rightFrame->addPoint( procPoint );

        }

    }

}

// ProcStereoFrame
ProcStereoFrame::ProcStereoFrame( const MapPtr &parent )
    : StereoFrame( parent )
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

void ProcStereoFrame::setMask( const StereoMat &value )
{
    leftFrame()->setMask( value.left() );
    rightFrame()->setMask( value.right() );
}

void ProcStereoFrame::extract()
{
    auto system = parentSystem();

    auto flowTracker = system->flowTracker();

    if ( flowTracker )
        flowTracker->extract( this );

    auto featureTracker = system->featureTracker();

    if ( featureTracker )
        featureTracker->extract( this );
}

void ProcStereoFrame::match()
{
    auto system = parentSystem();

    auto flowTracker = system->flowTracker();

    if ( flowTracker )
        flowTracker->match( this );

    auto featureTracker = system->featureTracker();

    if ( featureTracker )
        featureTracker->match( this );
}

size_t ProcStereoFrame::triangulatePoints()
{
    size_t ret = 0;

    auto size = _points.size();

    if ( size > 0 ) {

        auto system = parentSystem();

        auto maxReprojectionError = system->parameters().maxReprojectionError();

        cv::Mat homogeneousPoints3d;

        cv::Mat_< float > leftPoints( 2, size ), rightPoints( 2, size );

        for ( size_t i = 0; i < size; ++i ) {
            auto leftPoint = _points[ i ]->leftPoint()->undistortedPoint();
            auto rightPoint = _points[ i ]->rightPoint()->undistortedPoint();
            leftPoints.row( 0 ).col( i ) = leftPoint.x;
            leftPoints.row( 1 ).col( i ) = leftPoint.y;
            rightPoints.row( 0 ).col( i ) = rightPoint.x;
            rightPoints.row( 1 ).col( i ) = rightPoint.y;
        }

        auto leftProjectionMatrix = this->leftProjectionMatrix();
        auto rightProjectionMatrix = this->rightProjectionMatrix();

        cv::triangulatePoints( leftProjectionMatrix, rightProjectionMatrix, leftPoints, rightPoints, homogeneousPoints3d );

        auto minDisparity = system->parameters().minimumDisparity();

        for ( size_t i = 0; i < size; ++i ) {

            if ( cv::norm( _points[ i ]->leftPoint()->undistortedPoint() - _points[ i ]->rightPoint()->undistortedPoint() ) > minDisparity ) {

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

                    if ( std::abs( leftW ) > DOUBLE_EPS && std::abs( rightW ) > DOUBLE_EPS && leftW * w > 0 && rightW * w > 0 ) {

                        cv::Point2f leftReprojPt( leftReprojMat.at< double >( 0, 0 ) / leftW,
                                                  leftReprojMat.at< double >( 1, 0 ) / leftW );

                        cv::Point2f rightReprojPt( rightReprojMat.at< double >( 0, 0 ) / rightW,
                                                  rightReprojMat.at< double >( 1, 0 ) / rightW );

                        auto leftNorm = cv::norm( leftReprojPt - _points[ i ]->leftPoint()->undistortedPoint() );
                        auto rightNorm = cv::norm( rightReprojPt - _points[ i ]->rightPoint()->undistortedPoint() );

                        if ( leftNorm < maxReprojectionError && rightNorm < maxReprojectionError ) {
                            ++ret;
                            _points[ i ]->setPoint3d( pt );

                        }

                    }

                }

            }

        }

    }

    return ret;

}

size_t ProcStereoFrame::triangulateTracks()
{
    size_t ret = 0;

    auto system = parentSystem();

    auto maxReprojectionError = system->parameters().maxReprojectionError();
    auto minimumDisparity = system->parameters().minimumDisparity();
    auto minimumTriangulationDistance = system->parameters().minimumTriangulationDistance();

    std::vector< ProcPointPtr > leftPoints, rightPoints;

    auto leftFlowPoints = leftFrame()->flowPointsVector();
    auto leftFeaturePoints = leftFrame()->featurePointsVector();

    leftPoints.reserve( leftFlowPoints.size() + leftFeaturePoints.size() );

    leftPoints.insert( leftPoints.end(), leftFlowPoints.begin(), leftFlowPoints.end() );
    leftPoints.insert( leftPoints.end(), leftFeaturePoints.begin(), leftFeaturePoints.end() );

    auto rightFlowPoints = rightFrame()->flowPointsVector();
    auto rightFeaturePoints = rightFrame()->featurePointsVector();

    rightPoints.reserve( rightFlowPoints.size() + rightFeaturePoints.size() );

    rightPoints.insert( rightPoints.end(), rightFlowPoints.begin(), rightFlowPoints.end() );
    rightPoints.insert( rightPoints.end(), rightFeaturePoints.begin(), rightFeaturePoints.end() );

    using PointPair = std::pair< Point2Ptr, ProcPointPtr >;

    std::map< StereoFramePtr, std::vector< PointPair > > leftPointsMap, rightPointsMap;

    for ( auto &i : leftPoints ) {

        auto track = i->parentTrack();

        if ( track ) {

            auto startPoint = track->startPoint();

            if ( startPoint ) {

                if ( cv::norm( i->undistortedPoint() - startPoint->undistortedPoint() ) > minimumDisparity ) {

                    auto pointFrame = startPoint->parentFrame();

                    if ( pointFrame ) {

                        auto pointStereoFrame = pointFrame->parentStereoFrame();

                        if ( pointStereoFrame ) {

                            if ( cv::norm( pointStereoFrame->translation() - translation() ) > minimumTriangulationDistance )
                                leftPointsMap[ pointStereoFrame ].push_back( PointPair( startPoint, i ) );

                        }

                    }

                }

            }

        }

    }

    for ( auto &i : rightPoints ) {

        auto track = i->parentTrack();

        if ( track ) {

            auto startPoint = track->startPoint();

            if ( startPoint ) {

                if ( cv::norm( i->undistortedPoint() - startPoint->undistortedPoint() ) > minimumDisparity ) {

                    auto pointFrame = startPoint->parentFrame();

                    if ( pointFrame ) {

                        auto pointStereoFrame = pointFrame->parentStereoFrame();

                        if ( pointStereoFrame ) {

                            if ( cv::norm( pointStereoFrame->translation() - translation() ) > minimumTriangulationDistance )
                                rightPointsMap[ pointStereoFrame ].push_back( PointPair( startPoint, i ) );


                        }

                    }

                }

            }

        }

    }


    for ( auto &i : leftPointsMap ) {

        auto size = i.second.size();

        if ( size > 0 ) {
            cv::Mat homogeneousPoints3d;

            cv::Mat_< float > prevPoints( 2, size ), lastPoints( 2, size );

            for ( size_t j = 0; j < size; ++j ) {
                auto prevPoint = i.second[ j ].first->undistortedPoint();
                auto lastPoint = i.second[ j ].second->undistortedPoint();
                prevPoints.row( 0 ).col( j ) = prevPoint.x;
                prevPoints.row( 1 ).col( j ) = prevPoint.y;
                lastPoints.row( 0 ).col( j ) = lastPoint.x;
                lastPoints.row( 1 ).col( j ) = lastPoint.y;
            }

            auto prevProjectionMatrix = i.first->leftProjectionMatrix();
            auto lastProjectionMatrix = leftProjectionMatrix();

            cv::triangulatePoints( prevProjectionMatrix, lastProjectionMatrix, prevPoints, lastPoints, homogeneousPoints3d );

            for ( size_t j = 0; j < size; ++j ) {

                auto prevPoint = i.second[ j ].first;
                auto lastPoint = i.second[ j ].second;

                if ( cv::norm( prevPoint->undistortedPoint() - lastPoint->undistortedPoint() ) > minimumDisparity ) {

                    auto w = homogeneousPoints3d.at< float >( 3, j );

                    if ( std::abs( w ) > FLOAT_EPS ) {

                        auto x = homogeneousPoints3d.at< float >( 0, j ) / w;
                        auto y = homogeneousPoints3d.at< float >( 1, j ) / w;
                        auto z = homogeneousPoints3d.at< float >( 2, j ) / w;

                        auto pt = cv::Point3f( x, y, z );

                        cv::Mat pt4d( 4, 1, CV_64F );
                        homogeneousPoints3d.col( j ).convertTo( pt4d, CV_64F );

                        cv::Mat prevReprojMat = prevProjectionMatrix * pt4d;
                        cv::Mat lastReprojMat = lastProjectionMatrix * pt4d;

                        auto prevW = prevReprojMat.at< double >( 2, 0 );
                        auto lastW = lastReprojMat.at< double >( 2, 0 );

                        if ( std::abs( prevW ) > DOUBLE_EPS && std::abs( lastW ) > DOUBLE_EPS && prevW * w > 0 && lastW * w > 0 ) {

                            cv::Point2f prevReprojPt( prevReprojMat.at< double >( 0, 0 ) / prevW,
                                                      prevReprojMat.at< double >( 1, 0 ) / prevW );

                            cv::Point2f lastReprojPt( lastReprojMat.at< double >( 0, 0 ) / lastW,
                                                      lastReprojMat.at< double >( 1, 0 ) / lastW );

                            auto prevNorm = cv::norm( prevReprojPt - prevPoint->undistortedPoint() );
                            auto lastNorm = cv::norm( lastReprojPt - lastPoint->undistortedPoint() );

                            if ( prevNorm < maxReprojectionError && lastNorm < maxReprojectionError ) {
                                ++ret;

                                auto track = lastPoint->parentTrack();
                                auto mapPoint = track->mapPoint();

                                if ( !mapPoint )
                                    track->createMapPoint( ColorPoint3d( pt, leftFrame()->color( lastPoint->point2d() ) ) );
                                else
                                    mapPoint->setPoint( ColorPoint3d( pt, leftFrame()->color( lastPoint->point2d() ) ) );

                            }

                        }

                    }

                }

            }

        }

    }

    for ( auto &i : rightPointsMap ) {

        auto size = i.second.size();

        if ( size > 0 ) {
            cv::Mat homogeneousPoints3d;

            cv::Mat_< float > prevPoints( 2, size ), lastPoints( 2, size );

            for ( size_t j = 0; j < size; ++j ) {
                auto prevPoint = i.second[ j ].first->undistortedPoint();
                auto lastPoint = i.second[ j ].second->undistortedPoint();
                prevPoints.row( 0 ).col( j ) = prevPoint.x;
                prevPoints.row( 1 ).col( j ) = prevPoint.y;
                lastPoints.row( 0 ).col( j ) = lastPoint.x;
                lastPoints.row( 1 ).col( j ) = lastPoint.y;
            }

            auto prevProjectionMatrix = i.first->rightProjectionMatrix();
            auto lastProjectionMatrix = rightProjectionMatrix();

            cv::triangulatePoints( prevProjectionMatrix, lastProjectionMatrix, prevPoints, lastPoints, homogeneousPoints3d );

            for ( size_t j = 0; j < size; ++j ) {

                auto prevPoint = i.second[ j ].first;
                auto lastPoint = i.second[ j ].second;

                if ( cv::norm( prevPoint->undistortedPoint() - lastPoint->undistortedPoint() ) > minimumDisparity ) {

                    auto w = homogeneousPoints3d.at< float >( 3, j );

                    if ( std::abs( w ) > FLOAT_EPS ) {

                        auto x = homogeneousPoints3d.at< float >( 0, j ) / w;
                        auto y = homogeneousPoints3d.at< float >( 1, j ) / w;
                        auto z = homogeneousPoints3d.at< float >( 2, j ) / w;

                        auto pt = cv::Point3f( x, y, z );

                        cv::Mat pt4d( 4, 1, CV_64F );
                        homogeneousPoints3d.col( j ).convertTo( pt4d, CV_64F );

                        cv::Mat prevReprojMat = prevProjectionMatrix * pt4d;
                        cv::Mat lastReprojMat = lastProjectionMatrix * pt4d;

                        auto prevW = prevReprojMat.at< double >( 2, 0 );
                        auto lastW = lastReprojMat.at< double >( 2, 0 );

                        if ( std::abs( prevW ) > DOUBLE_EPS && std::abs( lastW ) > DOUBLE_EPS && prevW * w > 0 && lastW * w > 0 ) {

                            cv::Point2f prevReprojPt( prevReprojMat.at< double >( 0, 0 ) / prevW,
                                                      prevReprojMat.at< double >( 1, 0 ) / prevW );

                            cv::Point2f lastReprojPt( lastReprojMat.at< double >( 0, 0 ) / lastW,
                                                      lastReprojMat.at< double >( 1, 0 ) / lastW );

                            auto prevNorm = cv::norm( prevReprojPt - prevPoint->undistortedPoint() );
                            auto lastNorm = cv::norm( lastReprojPt - lastPoint->undistortedPoint() );

                            if ( prevNorm < maxReprojectionError && lastNorm < maxReprojectionError ) {
                                ++ret;

                                auto track = lastPoint->parentTrack();
                                auto mapPoint = track->mapPoint();

                                if ( !mapPoint )
                                    track->createMapPoint( ColorPoint3d( pt, leftFrame()->color( lastPoint->point2d() ) ) );
                                else
                                    mapPoint->setPoint( ColorPoint3d( pt, leftFrame()->color( lastPoint->point2d() ) ) );

                            }

                        }

                    }

                }

            }

        }

    }


    return ret;
}

double ProcStereoFrame::recoverPose()
{
    auto frame = leftFrame();

    auto points = frame->recoverPoints();

    auto system = parentSystem();

    auto maxReprojectionError = system->parameters().maxReprojectionError();
    auto minRecoverPointsCount = system->parameters().minimumRecoverPointsCount();

    std::vector< cv::Point3f > points3d;
    std::vector< cv::Point2f > points2d;

    for ( auto &i : points ) {

        auto track = i->parentTrack();
        auto mapPoint = track->mapPoint();

        points3d.push_back( mapPoint->point().point() );
        points2d.push_back( i->undistortedPoint() );

    }

    if ( points.size() < minRecoverPointsCount ) {
        qDebug() << "minRecoverPointsCount fail." << points.size();
        return 0.;
    }

    cv::Mat rvec;
    cv::Mat tvec;

    std::vector< int > inliers;

    if ( !cv::solvePnPRansac( points3d, points2d, frame->cameraMatrix(), cv::noArray(), rvec, tvec, false, 1000, maxReprojectionError, 0.9999, inliers, cv::SOLVEPNP_ITERATIVE ) ) {
        qDebug() << "solvePnPRansac fail";
        return 0.;
    }

    std::set< int > inliersSet;

    for ( auto i : inliers )
        inliersSet.insert( i );

    for ( size_t i = 0; i < points.size(); ++i )
        if ( inliersSet.find( i ) == inliersSet.end() )
            points[ i ]->parentTrack()->clearMapPoint();

    cv::Mat rmat;
    cv::Rodrigues( rvec, rmat );

    setTranslation( tvec );
    setRotation( rmat );

    return static_cast< double >( inliers.size() ) / points.size();

}

ProcFramePtr ProcStereoFrame::leftFrame()
{
    return std::dynamic_pointer_cast< ProcFrame >( StereoFrame::leftFrame() );
}

ProcFramePtr ProcStereoFrame::rightFrame()
{
    return std::dynamic_pointer_cast< ProcFrame >( StereoFrame::rightFrame() );
}

ProcFrameConstPtr ProcStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< const ProcFrame >( StereoFrame::leftFrame() );
}

ProcFrameConstPtr ProcStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< const ProcFrame >( StereoFrame::rightFrame() );
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

    auto stereoPoint = FlowStereoPoint::create();

    stereoPoint->set(  leftPoint, rightPoint  );

    _points.push_back( stereoPoint );

    return stereoPoint;
}

FeatureStereoPointPtr ProcStereoFrame::createFeaturePoint( const size_t leftIndex, const size_t rightIndex )
{
    auto leftPoint = leftFrame()->createFeaturePoint( leftIndex );
    auto rightPoint = rightFrame()->createFeaturePoint( rightIndex );

    auto stereoPoint = FeatureStereoPoint::create();

    stereoPoint->set( leftPoint, rightPoint );

    _points.push_back( stereoPoint );

    return stereoPoint;
}

CvImage ProcStereoFrame::drawPoints() const
{
    return leftFrame()->drawPoints();
}

CvImage ProcStereoFrame::drawTracks() const
{
    return leftFrame()->drawTracks();
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

        drawLine( &stackedImage, left, right + offset, 2, cv::Scalar( 0, 255, 0, 255 ) );
        drawFeaturePoint( &stackedImage, left, radius, cv::Scalar( 0, 0, 255, 255 ) );
        drawFeaturePoint( &stackedImage, right + offset, radius, cv::Scalar( 0, 0, 255, 255 ) );

    }

    drawLabel( &stackedImage, "Stereo count: " + std::to_string( _points.size() ), std::min( stackedImage.height(), stackedImage.width() ) * system->parameters().textDrawScale() );

    return stackedImage;
}

const std::vector< ProcStereoPointPtr > &ProcStereoFrame::stereoPoints() const
{
    return _points;
}

void ProcStereoFrame::setImagePyramid( const std::vector< cv::Mat > &leftPyramid, const std::vector< cv::Mat > &rightPyramid )
{
    leftFrame()->setImagePyramid( leftPyramid );
    rightFrame()->setImagePyramid( rightPyramid );
}

void ProcStereoFrame::setKeyPoints( const std::vector< cv::KeyPoint > &left, const std::vector< cv::KeyPoint > &right )
{
    leftFrame()->setKeyPoints( left );
    rightFrame()->setKeyPoints( right );
}

void ProcStereoFrame::setDescriptors( const cv::Mat &left, const cv::Mat &right )
{
    leftFrame()->setDescriptors( left );
    rightFrame()->setDescriptors( right );
}

// ConsecutiveFrame
ConsecutiveFrame::ConsecutiveFrame( const ProcFramePtr &frame1, const ProcFramePtr &frame2, const MapPtr &parent )
    : DoubleFrame( parent )
{
    set( frame1, frame2 );
}

ConsecutiveFrame::ObjectPtr ConsecutiveFrame::create( const ProcFramePtr &frame1, const ProcFramePtr &frame2, const MapPtr &parent )
{
    return ObjectPtr( new ConsecutiveFrame( frame1, frame2, parent ) );
}

ProcFramePtr ConsecutiveFrame::frame1()
{
    return std::dynamic_pointer_cast< ProcFrame >( DoubleFrame::frame1() );
}

ProcFramePtr ConsecutiveFrame::frame2()
{
    return std::dynamic_pointer_cast< ProcFrame >( DoubleFrame::frame2() );
}

ProcFrameConstPtr ConsecutiveFrame::frame1() const
{
    return std::dynamic_pointer_cast< const ProcFrame >( DoubleFrame::frame1() );
}

ProcFrameConstPtr ConsecutiveFrame::frame2() const
{
    return std::dynamic_pointer_cast< const ProcFrame >( DoubleFrame::frame2() );
}

void ConsecutiveFrame::extract()
{
    auto system = parentSystem();

    auto flowTracker = system->flowTracker();

    if ( flowTracker )
        flowTracker->extract( this );

    auto featureTracker = system->featureTracker();

    if ( featureTracker )
        featureTracker->extract( this );
}

void ConsecutiveFrame::track()
{
    auto system = parentSystem();

    auto flowTracker = system->flowTracker();

    if ( flowTracker )
        flowTracker->match( this );

    auto featureTracker = system->featureTracker();

    if ( featureTracker )
        featureTracker->match( this );
}

// ConsecutiveFrame
ConsecutiveStereoFrame::ConsecutiveStereoFrame( const ProcStereoFramePtr &prevFrame, const ProcStereoFramePtr &nextFrame )
{
    set( prevFrame, nextFrame );
}

void ConsecutiveStereoFrame::set( const ProcStereoFramePtr &prevFrame, const ProcStereoFramePtr &nextFrame )
{
    _prevFrame = prevFrame;
    _nextFrame = nextFrame;
}

const ProcStereoFramePtr &ConsecutiveStereoFrame::prevFrame() const
{
    return _prevFrame;
}

const ProcStereoFramePtr &ConsecutiveStereoFrame::nextFrame() const
{
    return _nextFrame;
}

}
