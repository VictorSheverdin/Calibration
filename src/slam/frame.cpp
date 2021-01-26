#include "src/common/precompiled.h"

#include "frame.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

#include "map.h"
#include "world.h"

#include "optimizer.h"

#include <opencv2/sfm.hpp>

namespace slam {

// MonoFrame
std::vector< cv::Point2f > MonoFrame::points() const
{
    std::vector< cv::Point2f > ret;

    auto framePoints = this->framePoints();

    for ( auto &i : framePoints )
        if ( i )
            ret.push_back( i->point() );

    return ret;

}

std::vector< StereoPoint > MonoFrame::stereoPoints() const
{
    std::vector< StereoPoint > ret;

    auto framePoints = this->framePoints();

    for ( auto &i : framePoints ) {

        if ( i && i->stereoPoint() )

            ret.push_back( StereoPoint( i, i->stereoPoint() ));
    }

    return ret;
}

// ProcessedFrame
const double ProcessedFrame::m_densityFactor = 4.;

ProcessedFrame::ProcessedFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
    initialize();
}

void ProcessedFrame::initialize()
{
    m_desiredPointsCount = 1000;
}

MapPtr ProcessedFrame::parentMap() const
{
    return m_parentMap.lock();
}

WorldPtr ProcessedFrame::parentWorld() const
{
    return parentMap()->parentWorld();
}

void ProcessedFrame::setImage( const StampedImage &image )
{
    m_image = image;
}

const CvImage &ProcessedFrame::image() const
{
    return m_image;
}

const cv::Mat &ProcessedFrame::mask() const
{
    return m_mask;
}

void ProcessedFrame::clearImage()
{
    m_image.release();
}

void ProcessedFrame::clearMask()
{
    m_mask.release();
}

CvImage ProcessedFrame::drawTracks() const
{
    CvImage ret;

    if ( !m_image.empty() ) {

        m_image.copyTo( ret );

        auto points = this->framePoints();

#pragma omp parallel for
        for ( size_t i = 0; i < points.size(); ++i )
            if ( points[ i ] && ( points[ i ]->prevPoint() || points[i]->nextPoint() ) ) {
                if ( points[ i ]->mapPoint() )
                    points[ i ]->drawTrack( &ret );
                else
                    points[ i ]->drawTrack( &ret, cv::Scalar( 255, 0, 0, 255 ) );

            }

        int radius = std::min( ret.width(), ret.height() ) / 500.0;

#pragma omp parallel for
        for ( size_t i = 0; i < points.size(); ++i )
            if ( points[ i ] && ( points[ i ]->prevPoint() || points[ i ]->nextPoint() ) )
                drawFeaturePoint( &ret, points[ i ]->point(), radius );

        drawLabel( &ret, "Tracks count: " + std::to_string( points.size() ), ret.height() / 70 );

    }

    return ret;

}

void ProcessedFrame::cleanMapPoints()
{
    auto parentMap = this->parentMap();

    auto points = framePoints();

    for ( auto &i : points ) {

        if ( i ) {

            auto mapPoint = i->mapPoint();

            if ( mapPoint ) {

                if ( !i->nextPoint() && i->connectedPointsCount() < m_minConnectedPoints ) {

                    parentMap->removeMapPoint( mapPoint );

                }

            }

        }

    }

}

void ProcessedFrame::removeExtraPoints( const size_t mapPointsCount, const size_t framePointsCount )
{
    size_t curMapPointsCount = 0;
    size_t curFramePointsCount = 0;

    auto framePoints = this->framePoints();

    for ( auto & i : framePoints ) {

        if ( i->mapPoint() )
            ++curMapPointsCount;
        else
            ++curFramePointsCount;

        if ( ( curMapPointsCount >= mapPointsCount && i->mapPoint() ) || ( curFramePointsCount >= framePointsCount && !i->mapPoint() ) )
            removePoint( i );

    }

}

std::vector< MonoPointPtr > ProcessedFrame::posePoints() const
{
    std::vector< MonoPointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        if ( i && i->prevPoint() && i->mapPoint() )
            ret.push_back( i );

    }

    return ret;

}

size_t ProcessedFrame::posePointsCount() const
{
    return posePoints().size();
}

std::vector< MonoPointPtr > ProcessedFrame::trackedPoints() const
{
    std::vector< MonoPointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        if ( i && i->nextPoint() && ( i->mapPoint() || i->stereoPoint() ) )
            ret.push_back( i );

    }

    return ret;

}

size_t ProcessedFrame::trackedPointsCount() const
{
    return trackedPoints().size();
}

void ProcessedFrame::setImagePyramid( const std::vector<cv::Mat> &value )
{
    m_imagePyramid = value;
}

const std::vector< cv::Mat > &ProcessedFrame::imagePyramid() const
{
    return m_imagePyramid;
}

const std::chrono::time_point< std::chrono::system_clock > &ProcessedFrame::time() const
{
    return m_image.time();
}

void ProcessedFrame::setDesiredPointsCount( const size_t value )
{
    m_desiredPointsCount = value;
}

size_t ProcessedFrame::desiredPointsCount() const
{
    return m_desiredPointsCount;
}

double ProcessedFrame::pointsInterval() const
{
    if ( m_image.rows > 0 && m_desiredPointsCount > 0 )
        return m_image.cols / sqrt( static_cast< double >( m_desiredPointsCount * m_image.cols ) / m_image.rows ) / m_densityFactor;
    else
        return 0.;

}

// ProcessedKeyFrame
ProcessedKeyFrame::ProcessedKeyFrame( const MapPtr &parentMap )
    : ProcessedFrame( parentMap )
{
}

// FlowFrame
FlowFrame::FlowFrame( const MapPtr &parentMap)
    : ProcessedFrame( parentMap )
{
    initialize();
}

void FlowFrame::initialize()
{
}

FlowFrame::ObjectPtr FlowFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FlowFrame( parentMap ) );
}

void FlowFrame::setImage( const StampedImage &image )
{
    ProcessedFrame::setImage( image );
}

void FlowFrame::buildPyramid()
{
    parentWorld()->flowTracker()->buildPyramid( this );
}

void FlowFrame::clearPyramid()
{
    m_imagePyramid.clear();
}

std::vector< MonoPointPtr > FlowFrame::framePoints() const
{
    return std::vector< MonoPointPtr >( m_flowPoints.begin(), m_flowPoints.end() );
}

size_t FlowFrame::framePointsCount() const
{
    return m_flowPoints.size();
}

void FlowFrame::removePoint( const MonoPointPtr &point )
{
    if ( point )
        m_flowPoints.erase( std::dynamic_pointer_cast< FlowPoint >( point ) );
}

std::vector< FlowPointPtr > FlowFrame::flowPoints() const
{
    return std::vector< FlowPointPtr >( m_flowPoints.begin(), m_flowPoints.end() );
}

size_t FlowFrame::flowPointsCount() const
{
    return m_flowPoints.size();
}

void FlowFrame::addFlowPoints( const std::vector< cv::Point2f > &vector )
{
    for ( auto &i : vector )
        addFlowPoint( i );
}

FlowPointPtr FlowFrame::addFlowPoint( const cv::Point2f &point )
{
    auto flowPoint = FlowPoint::create( shared_from_this(), point, m_image.at< cv::Vec3b >( point ) );

    flowPoint->setError( 0. );

    m_flowPoints.insert( flowPoint );

    return flowPoint;

}

FlowFrame::ObjectPtr FlowFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FlowFrame >( MonoFrame::shared_from_this() );
}

FlowFrame::ObjectConstPtr FlowFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FlowFrame >( MonoFrame::shared_from_this() );
}

cv::Mat track( const std::shared_ptr< FlowFrame > &prevFrame, const std::shared_ptr< FlowFrame > &nextFrame )
{
    if ( prevFrame && nextFrame ) {

        std::vector< FlowTrackResult > trackedPoints;

        auto fmat = prevFrame->parentWorld()->flowTracker()->track( prevFrame, nextFrame, &trackedPoints );

        auto trackPoints = prevFrame->flowPoints();

        for ( auto &i : trackedPoints ) {

            auto prevPoint = trackPoints[ i.index ];
            auto nextPoint = nextFrame->addFlowPoint( i );

            prevPoint->setNextPoint( nextPoint );
            nextPoint->setPrevPoint( prevPoint );

            auto worlPoint = prevPoint->mapPoint();

            if ( worlPoint )
                nextPoint->setMapPoint( worlPoint );

            nextPoint->setError( prevPoint->error() + i.error );
            nextPoint->setMisstake( prevPoint->misstake() + i.miss );

        }

        return fmat;

    }

    return cv::Mat();

}

// FlowKeyFrame
FlowKeyFrame::FlowKeyFrame( const MapPtr &parentMap )
    : ProcessedFrame( parentMap ), FlowFrame( parentMap ), ProcessedKeyFrame( parentMap )
{
    initialize();
}

void FlowKeyFrame::initialize()
{
}

FlowKeyFrame::ObjectPtr FlowKeyFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FlowKeyFrame( parentMap ) );
}

void FlowKeyFrame::setImage( const StampedImage &image )
{
    FlowFrame::setImage( image );
}

void FlowKeyFrame::extractPoints()
{
    parentWorld()->flowTracker()->extractPoints( this );
}

void FlowKeyFrame::createMask()
{
    if ( !m_image.empty() ) {
        m_mask = cv::Mat( m_image.rows, m_image.cols, CV_8U, cv::Scalar( 1 ) );

        auto distance = pointsInterval();

        for ( auto &i : m_flowPoints ) {
            if ( i ) {
                auto pt = i->point();
                cv::rectangle( m_mask, cv::Rect( pt.x - distance, pt.y - distance, distance * 2, distance * 2 ), cv::Scalar( 0 ) );
            }

        }

    }
    else
        m_mask.release();
}

void FlowKeyFrame::replace( const FlowFramePtr &frame )
{
    if ( frame ) {

        setImage( frame->image() );
        setImagePyramid( frame->imagePyramid() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            if ( i ) {
                auto point = addFlowPoint( i->point() );
                point->replace( i );

            }

        }

    }

}

void FlowKeyFrame::replaceAndClean( const FlowFramePtr &frame )
{
    if ( frame ) {

        setImage( frame->image() );
        setImagePyramid( frame->imagePyramid() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            if ( i && ( i->mapPoint() || i->nextPoint() ) ) {
                auto point = addFlowPoint( i->point() );
                point->replace( i );

            }

        }

    }

}

FlowKeyFrame::ObjectPtr FlowKeyFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FlowKeyFrame >( MonoFrame::shared_from_this() );
}

FlowKeyFrame::ObjectConstPtr FlowKeyFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FlowKeyFrame >( MonoFrame::shared_from_this() );
}
/*
// FeatureFrame
FeatureFrame::FeatureFrame( const FeatureMapPtr &parentMap )
    : ProcessedFrame( parentMap )
{
}

std::vector< MonoPointPtr > FeatureFrame::framePoints() const
{
    std::vector< MonoPointPtr > ret;

    for ( auto &i : m_points )
        ret.push_back( i.second );

    return ret;

}

size_t FeatureFrame::framePointsCount() const
{
    return m_points.size();
}

void FeatureFrame::removePoint( const MonoPointPtr &point )
{
    for ( auto i = m_points.begin(); i != m_points.end(); ++i )
        if ( i->second == point )
            i = m_points.erase( i );
}

std::vector< FeaturePointPtr > FeatureFrame::featurePoints() const
{
    std::vector< FeaturePointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        auto processedPoint = std::dynamic_pointer_cast< FeaturePoint >( i );

        if ( processedPoint )
            ret.push_back( processedPoint );

    }

    return ret;

}

FeaturePointPtr &FeatureFrame::featurePoint( const size_t index )
{
    if ( !isFramePointExist( index ) )
        createFramePoint( index );

    return m_points.at( index );
}

const FeaturePointPtr &FeatureFrame::featurePoint( const size_t index ) const
{
    return m_points.at( index );
}

FeatureFrame::ObjectPtr FeatureFrame::create( const FeatureMapPtr &parentMap )
{
    return ObjectPtr( new FeatureFrame( parentMap ) );
}

void FeatureFrame::setImage( const StampedImage &image )
{
    ProcessedFrame::setImage( image );

    m_keyPoints.clear();
    m_descriptors.release();
    m_points.clear();
    m_colors.clear();

    extractKeypoints();

}

void FeatureFrame::extractKeypoints()
{
    parentWorld()->featureTracker()->extractKeypoints( this );

    for ( size_t i = 0; i < m_keyPoints.size(); ++i )
        m_colors.push_back( m_image.at< cv::Vec3b >( m_keyPoints[ i ].pt ) );

}

void FeatureFrame::createMask()
{
    if ( !m_image.empty() ) {
        m_mask = cv::Mat( m_image.rows, m_image.cols, CV_8U, cv::Scalar( 1 ) );

        auto distance = pointsInterval();

        for ( auto &i : m_points ) {
            if ( i.second ) {
                auto pt = i.second->point();
                cv::rectangle( m_mask, cv::Rect( pt.x - distance, pt.y - distance, distance * 2, distance * 2 ), cv::Scalar( 0 ) );
            }

        }

    }
    else
        m_mask.release();

}

FeaturePointPtr FeatureFrame::createFramePoint( const size_t keyPointIndex )
{
    if ( keyPointIndex >= m_keyPoints.size() )
        return FeaturePointPtr();

    auto point = FeaturePoint::create( shared_from_this(), keyPointIndex );

    m_points[ keyPointIndex ] = point;

    return point;

}

bool FeatureFrame::isFramePointExist( const size_t index ) const
{
    return m_points.find( index ) != m_points.end();
}

void FeatureFrame::createFramePoints( const size_t count )
{
    if ( count > 0 ) {

        size_t addCount = 0;

        for ( size_t i = 0; i < m_keyPoints.size(); ++i ) {
            if ( !isFramePointExist( i ) ) {
                createFramePoint( i );
                ++addCount;

                if ( addCount > count )
                    return;

            }

        }

    }

}

const std::vector< cv::KeyPoint > &FeatureFrame::keyPoints() const
{
    return m_keyPoints;
}

void FeatureFrame::setKeyPoints( const std::vector<cv::KeyPoint> &value )
{
    m_keyPoints = value;
}

const cv::Mat &FeatureFrame::descriptors() const
{
    return m_descriptors;
}

void FeatureFrame::setDescriptors( const cv::Mat &value )
{
    m_descriptors = value;
}

std::shared_ptr< FeatureFrame > FeatureFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FeatureFrame >( MonoFrame::shared_from_this() );
}

std::shared_ptr< const FeatureFrame > FeatureFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FeatureFrame >( MonoFrame::shared_from_this() );
}

cv::Mat track( const std::shared_ptr< FeatureFrame > &prevFrame, const std::shared_ptr< FeatureFrame > &nextFrame )
{

    if ( prevFrame && nextFrame ) {

        auto trackPoints = prevFrame->featurePoints();

        std::vector< cv::DMatch > matches;

        auto fmat = prevFrame->parentWorld()->featureTracker()->match( prevFrame, nextFrame, &matches );

        for ( auto &i : matches ) {

            auto prevPoint = trackPoints[ i.queryIdx ];
            auto nextPoint = nextFrame->featurePoint( i.trainIdx );

            prevPoint->setNextPoint( nextPoint );
            nextPoint->setPrevPoint( prevPoint );

            auto worlPoint = prevPoint->mapPoint();

            if ( worlPoint )
                nextPoint->setMapPoint( worlPoint );

        }

        return fmat;

    }

    return cv::Mat();

}
*/
// FinishedFrame
FinishedFrame::FinishedFrame( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    setTime( time );
}

FinishedFrame::ObjectPtr FinishedFrame::create( const std::chrono::time_point< std::chrono::system_clock > time )
{
    return ObjectPtr( new FinishedFrame( time ) );
}

void FinishedFrame::setTime( const std::chrono::time_point<std::chrono::system_clock> &value )
{
    m_time = value;
}

const std::chrono::time_point<std::chrono::system_clock> &FinishedFrame::time() const
{
    return m_time;
}

std::vector< MonoPointPtr > FinishedFrame::framePoints() const
{
    std::vector< MonoPointPtr > ret;

    for ( auto &i : m_points )
        ret.push_back( i );

    return ret;
}

size_t FinishedFrame::framePointsCount() const
{
    return m_points.size();
}

FinishedFramePointPtr FinishedFrame::createFramePoint( const cv::Point2f &point, const cv::Scalar &color )
{
    auto ret = FinishedFramePoint::create( shared_from_this(), point, color );

    m_points.push_back( ret );

    return ret;

}

FinishedFrame::ObjectPtr FinishedFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinishedFrame >( MonoFrame::shared_from_this() );
}

FinishedFrame::ObjectConstPtr FinishedFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinishedFrame >( MonoFrame::shared_from_this() );
}

void FinishedFrame::replace( const ProcessedFramePtr &frame )
{
    if ( frame ) {

        setTime( frame->time() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            if ( i ) {
                auto point = createFramePoint( i->point(), i->color() );
                point->replace( i );

            }

        }

    }

}

void FinishedFrame::replaceAndClean( const ProcessedFramePtr &frame )
{
    if ( frame ) {

        setTime( frame->time() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            if ( i && ( i->mapPoint() || i->nextPoint() ) ) {
                auto point = createFramePoint( i->point(), i->color() );
                point->replace( i );

            }

        }

    }

}


// FinishedKeyFrame
FinishedKeyFrame::FinishedKeyFrame( const std::chrono::time_point< std::chrono::system_clock > &time )
    : FinishedFrame( time )
{
}

FinishedKeyFrame::ObjectPtr FinishedKeyFrame::create( const std::chrono::time_point< std::chrono::system_clock > time )
{
    return ObjectPtr( new FinishedKeyFrame( time ) );
}


void FinishedKeyFrame::replace( const ProcessedKeyFramePtr &frame )
{
    if ( frame ) {

        setTime( frame->time() );

        setProjectionMatrix( frame->projectionMatrix() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            if ( i ) {
                auto point = createFramePoint( i->point(), i->color() );
                point->replace( i );

            }

        }

    }

}

void FinishedKeyFrame::replaceAndClean( const ProcessedKeyFramePtr &frame )
{
    if ( frame ) {

        setTime( frame->time() );

        setProjectionMatrix( frame->projectionMatrix() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            if ( i && ( i->mapPoint() || i->nextPoint() ) ) {
                auto point = createFramePoint( i->point(), i->color() );
                point->replace( i );

            }

        }

    }

}

// DoubleFrame
DoubleFrame::DoubleFrame()
{
}

void DoubleFrame::setFrame1( const MonoFramePtr &frame )
{
    m_frame1 = frame;
}

void DoubleFrame::setFrame2( const MonoFramePtr &frame )
{
    m_frame2 = frame;
}

void DoubleFrame::setFrames( const MonoFramePtr &frame1, const MonoFramePtr &frame2 )
{
    setFrame1( frame1 );
    setFrame2( frame2 );
}

MonoFramePtr DoubleFrame::frame1() const
{
    return m_frame1;
}

MonoFramePtr DoubleFrame::frame2() const
{
    return m_frame2;
}

//StereoFrame
StereoFrame::StereoFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
}

void StereoFrame::setLeftFrame( const MonoFramePtr &frame )
{
    DoubleFrame::setFrame1( frame );
}

void StereoFrame::setRightFrame( const MonoFramePtr &frame )
{
    DoubleFrame::setFrame2( frame );
}

void StereoFrame::setFrames( const MonoFramePtr &leftFrame, const MonoFramePtr &rightFrame )
{
    DoubleFrame::setFrames( leftFrame, rightFrame );
}

std::vector< StereoPoint > StereoFrame::stereoPoints() const
{
    auto leftFrame = this->leftFrame();

    if ( leftFrame )
        return leftFrame->stereoPoints();
    else
        return std::vector< StereoPoint >();

}

size_t StereoFrame::stereoPointsCount() const
{
    return stereoPoints().size();
}

MonoFramePtr StereoFrame::leftFrame() const
{
    return frame1();
}

MonoFramePtr StereoFrame::rightFrame() const
{
    return frame2();
}

MapPtr StereoFrame::parentMap() const
{
    return m_parentMap.lock();
}

WorldPtr StereoFrame::parentWorld() const
{
    return parentMap()->parentWorld();
}

// ProcessedStereoFrame
ProcessedStereoFrame::ProcessedStereoFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap )
{
}

ProcessedFramePtr ProcessedStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( StereoFrame::leftFrame() );
}

ProcessedFramePtr ProcessedStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( StereoFrame::rightFrame() );
}

void ProcessedStereoFrame::clearImages()
{
    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame )
        leftFeatureFrame->clearImage();

    if ( rightFeatureFrame )
        rightFeatureFrame->clearImage();

}

CvImage ProcessedStereoFrame::drawStereoCorrespondences() const
{
    CvImage ret;

    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame && rightFeatureFrame ) {

        auto leftImage = leftFeatureFrame->image();
        auto rightImage = rightFeatureFrame->image();

        if ( !leftImage.empty() && !rightImage.empty() ) {

            ret = stackImages( leftImage, rightImage );

            auto stereoPoints = this->stereoPoints();

#pragma omp parallel for
            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {

                    auto rightPoint = stereoPoints[ i ].rightPoint();

                    rightPoint.x += leftImage.width();

                    drawLine( &ret, stereoPoints[ i ].leftPoint(), rightPoint );


            }

            int radius = std::min( ret.width(), ret.height() ) / 500.0;

#pragma omp parallel for
            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {
                auto rightPoint = stereoPoints[ i ].rightPoint();
                rightPoint.x += leftImage.width();

                drawFeaturePoint( &ret, stereoPoints[ i ].leftPoint(), radius );
                drawFeaturePoint( &ret, rightPoint, radius );

            }

            drawLabel( &ret, "Stereo correspondencies count: " + std::to_string( stereoPoints.size() ), ret.height() / 70 );


        }

    }

    return ret;

}

CvImage ProcessedStereoFrame::drawTracks() const
{
    CvImage leftImage;
    CvImage rightImage;

    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame )
        leftImage = leftFeatureFrame->drawTracks();

    if ( rightFeatureFrame )
        rightImage = rightFeatureFrame->drawTracks();

    return stackImages( leftImage, rightImage );

}

void ProcessedStereoFrame::removeExtraPoints( const size_t mapPointsCount, const size_t framePointsCount )
{
    auto leftFrame = this->leftFrame();

    if ( leftFrame )
        leftFrame->removeExtraPoints( mapPointsCount, framePointsCount );

}

void ProcessedStereoFrame::cleanMapPoints()
{
    auto leftFrame = this->leftFrame();

    if ( leftFrame )
        leftFrame->cleanMapPoints();

}

// StereoKeyFrame
StereoKeyFrame::StereoKeyFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap )
{
}

MonoKeyFramePtr StereoKeyFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< MonoKeyFrame >( StereoFrame::leftFrame() );
}

MonoKeyFramePtr StereoKeyFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< MonoKeyFrame >( StereoFrame::rightFrame() );
}

void StereoKeyFrame::setProjectionMatrix( const ProjectionMatrix &matrix1, const ProjectionMatrix &matrix2 )
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame )
        leftFrame->setProjectionMatrix( matrix1 );

    if ( rightFrame )
        rightFrame->setProjectionMatrix( matrix2 );
}

void StereoKeyFrame::setProjectionMatrix( const StereoCameraMatrix &matrix )
{
    leftFrame()->setProjectionMatrix( matrix.leftProjectionMatrix() );
    rightFrame()->setProjectionMatrix( matrix.rightProjectionMatrix() );
}

StereoCameraMatrix StereoKeyFrame::projectionMatrix() const
{
    return StereoCameraMatrix( leftFrame()->projectionMatrix(), rightFrame()->projectionMatrix() );
}

double StereoKeyFrame::bf() const
{
    return leftFrame()->fx() * cv::norm( leftFrame()->translation() - rightFrame()->translation() );
}

void StereoKeyFrame::setLeftSe3Pose( g2o::SE3Quat &pose )
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        auto rightPose = pose;
        rightPose.setTranslation( pose.translation() + rightFrame->se3Pose().translation() - leftFrame->se3Pose().translation() );

        leftFrame->setSe3Pose( pose );
        rightFrame->setSe3Pose( rightPose );

    }

}

cv::Point3f StereoKeyFrame::center() const
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame )
        return 0.5 * ( static_cast< cv::Point3d >( *leftFrame ) + static_cast< cv::Point3d >( *rightFrame ) );

    return cv::Point3f();
}

int StereoKeyFrame::triangulatePoints()
{
    int ret = 0;

    auto map = parentMap();

    if ( map ) {

        auto maxReprojectionError = parentWorld()->maxReprojectionError();

        cv::Mat homogeneousPoints3d;

        auto stereoPoints = this->stereoPoints();

        cv::Mat_< float > leftPoints( 2, stereoPoints.size() ),
                           rightPoints( 2, stereoPoints.size() );

        for ( size_t i = 0; i < stereoPoints.size(); ++i ) {

            leftPoints.row(0).col(i) = stereoPoints[i].leftPoint().x;
            leftPoints.row(1).col(i) = stereoPoints[i].leftPoint().y;
            rightPoints.row(0).col(i) = stereoPoints[i].rightPoint().x;
            rightPoints.row(1).col(i) = stereoPoints[i].rightPoint().y;

        }

        if ( !leftPoints.empty() && leftPoints.size() == rightPoints.size() ) {

            auto leftProjectionMatrix = leftFrame()->projectionMatrix();
            auto rightProjectionMatrix = rightFrame()->projectionMatrix();

            cv::triangulatePoints( leftProjectionMatrix, rightProjectionMatrix, leftPoints, rightPoints, homogeneousPoints3d );

            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {

                auto w = homogeneousPoints3d.at< float >( 3, i );

                if ( std::abs( w ) > FLOAT_EPS ) {

                    auto x = homogeneousPoints3d.at< float >( 0, i ) / w;
                    auto y = homogeneousPoints3d.at< float >( 1, i ) / w;
                    auto z = homogeneousPoints3d.at< float >( 2, i ) / w;

                    auto pt = cv::Point3d( x, y, z );

                    cv::Mat pt4d( 4, 1, CV_64F );
                    homogeneousPoints3d.col( i ).convertTo( pt4d, CV_64F );

                    cv::Mat leftReprojMat = leftProjectionMatrix * pt4d;
                    cv::Mat rightReprojMat = rightProjectionMatrix * pt4d;

                    auto leftW = leftReprojMat.at< double >( 2, 0 );
                    auto rightW = rightReprojMat.at< double >( 2, 0 );

                    if ( std::abs( leftW ) > DOUBLE_EPS && std::abs( rightW ) > DOUBLE_EPS &&
                            leftW / w > 0 && rightW / w > 0 ) {

                        cv::Point2d leftReprojPt( leftReprojMat.at< double >( 0, 0 ) / leftW,
                                                  leftReprojMat.at< double >( 1, 0 ) / leftW );

                        cv::Point2d rightReprojPt( rightReprojMat.at< double >( 0, 0 ) / rightW,
                                                  rightReprojMat.at< double >( 1, 0 ) / rightW );

                        auto leftNorm = cv::norm( leftReprojPt - cv::Point2d( stereoPoints[i].leftPoint() ) );
                        auto rightNorm = cv::norm( rightReprojPt - cv::Point2d( stereoPoints[i].rightPoint() ) );

                        if ( leftNorm < maxReprojectionError && rightNorm < maxReprojectionError ) {

                            ++ret;

                            auto leftFramePoint = stereoPoints[i].leftFramePoint();
                            auto rightFramePoint = stereoPoints[i].rightFramePoint();

                            MapPointPtr mapPoint;

                            if ( !leftFramePoint->mapPoint() && !rightFramePoint->mapPoint() ) {

                                mapPoint = map->createMapPoint( pt, leftFramePoint->color() );

                                leftFramePoint->setMapPoint( mapPoint );
                                rightFramePoint->setMapPoint( mapPoint );

                            }
                            else {

                                if ( !stereoPoints[i].leftFramePoint()->mapPoint() )
                                    leftFramePoint->setMapPoint( rightFramePoint->mapPoint() );

                                else if ( !rightFramePoint->mapPoint() )
                                    rightFramePoint->setMapPoint( leftFramePoint->mapPoint() );

                                mapPoint = leftFramePoint->mapPoint();

                                mapPoint->setPoint( pt );
                                mapPoint->setColor( leftFramePoint->color() );

                            }

                            auto leftNextPoint = leftFramePoint->nextPoint();

                            if ( leftNextPoint )
                                leftNextPoint->setMapPoint( mapPoint );

                            auto rightNextPoint = rightFramePoint->nextPoint();

                            if ( rightNextPoint )
                                rightNextPoint->setMapPoint( mapPoint );

                        }

                    }

                }

            }

        }

    }

    return ret;

}

// ProcessedStereoKeyFrame
ProcessedStereoKeyFrame::ProcessedStereoKeyFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap )
{
}

ProcessedKeyFramePtr ProcessedStereoKeyFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< ProcessedKeyFrame >( StereoFrame::leftFrame() );
}

ProcessedKeyFramePtr ProcessedStereoKeyFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcessedKeyFrame >( StereoFrame::rightFrame() );
}

// FlowStereoFrame
FlowStereoFrame::FlowStereoFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap )
{
    setFrames( FlowFrame::create( parentMap ), FlowFrame::create( parentMap ) );
}

FlowStereoFrame::ObjectPtr FlowStereoFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FlowStereoFrame( parentMap ) );
}

void FlowStereoFrame::setLeftImage( const StampedImage &image )
{
    auto frame = leftFrame();

    if ( frame )
        frame->setImage( image );
}

void FlowStereoFrame::setRightImage( const StampedImage &image )
{
    auto frame = rightFrame();

    if ( frame )
        frame->setImage( image );
}

void FlowStereoFrame::setImage( const StampedImage &leftImage, const StampedImage &rightImage )
{
    setLeftImage( leftImage );
    setRightImage( rightImage );
}

void FlowStereoFrame::buildPyramid()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame )
        leftFrame->buildPyramid();

    if ( rightFrame )
        rightFrame->buildPyramid();
}

void FlowStereoFrame::clearPyramid()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame )
        leftFrame->clearPyramid();

    if ( rightFrame )
        rightFrame->clearPyramid();
}

FlowFramePtr FlowStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FlowFrame >( StereoFrame::leftFrame() );
}

FlowFramePtr FlowStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FlowFrame >( StereoFrame::rightFrame() );
}

cv::Mat FlowStereoFrame::match()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        std::vector< FlowTrackResult > trackedPoints;

        auto fmat = parentWorld()->flowTracker()->track( leftFrame, rightFrame, &trackedPoints );

        auto trackPoints = leftFrame->flowPoints();

        auto minDistance = parentWorld()->minStereoDisparity();

        for ( auto &i : trackedPoints ) {

            auto leftFlowPoint = trackPoints[ i.index ];
            auto leftPoint = leftFlowPoint->point();
            auto rightPoint = i;

            if ( cv::norm( leftPoint - rightPoint ) > minDistance ) {

                auto rightFlowPoint = rightFrame->addFlowPoint( rightPoint );

                leftFlowPoint->setStereoPoint( rightFlowPoint );
                rightFlowPoint->setStereoPoint( leftFlowPoint );

                auto worlPoint = leftFlowPoint->mapPoint();

                if ( worlPoint )
                    rightFlowPoint->setMapPoint( worlPoint );

                rightFlowPoint->setError( leftFlowPoint->error() + i.error );
                rightFlowPoint->setMisstake( leftFlowPoint->misstake() + i.miss );

            }

        }

        return fmat;

    }

    return cv::Mat();

}

// FlowStereoKeyFrame
FlowStereoKeyFrame::FlowStereoKeyFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap ), ProcessedStereoKeyFrame( parentMap ), FlowStereoFrame( parentMap )
{
    setFrames( FlowKeyFrame::create( parentMap ), FlowKeyFrame::create( parentMap ) );
}

FlowStereoKeyFrame::ObjectPtr FlowStereoKeyFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FlowStereoKeyFrame( parentMap ) );
}

void FlowStereoKeyFrame::extractPoints()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame )
        leftFrame->extractPoints();

    if ( rightFrame )
        rightFrame->extractPoints();
}

FlowKeyFramePtr FlowStereoKeyFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FlowKeyFrame >( StereoFrame::leftFrame() );
}

FlowKeyFramePtr FlowStereoKeyFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FlowKeyFrame >( StereoFrame::rightFrame() );
}

void FlowStereoKeyFrame::replace( const FlowStereoFramePtr &frame )
{
    if ( frame ) {

        auto parentMap = this->parentMap();

        auto leftFrame = this->leftFrame();

        if ( !leftFrame ) {
            leftFrame = FlowKeyFrame::create( parentMap );
            setLeftFrame( leftFrame );
        }

        auto rightFrame = this->rightFrame();

        if ( !rightFrame ) {
            rightFrame = FlowKeyFrame::create( parentMap);
            setRightFrame( rightFrame );
        }

        auto leftSourceFrame = frame->leftFrame();
        auto rightSourceFrame = frame->rightFrame();

        if ( leftFrame && leftSourceFrame )
            leftFrame->replace( leftSourceFrame );

        if ( rightFrame && rightSourceFrame )
            rightFrame->replace( rightSourceFrame );

    }

}

void FlowStereoKeyFrame::replaceAndClean( const FlowStereoFramePtr &frame )
{
    if ( frame ) {

        auto parentMap = this->parentMap();

        auto leftFrame = this->leftFrame();

        if ( !leftFrame ) {
            leftFrame = FlowKeyFrame::create( parentMap );
            setLeftFrame( leftFrame );
        }

        auto rightFrame = this->rightFrame();

        if ( !rightFrame ) {
            rightFrame = FlowKeyFrame::create( parentMap);
            setRightFrame( rightFrame );
        }

        auto leftSourceFrame = frame->leftFrame();
        auto rightSourceFrame = frame->rightFrame();

        if ( leftFrame && leftSourceFrame )
            leftFrame->replaceAndClean( leftSourceFrame );

        if ( rightFrame && rightSourceFrame )
            rightFrame->replaceAndClean( rightSourceFrame );

    }

}

// DenseFrame
DenseFrame::DenseFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), StereoKeyFrame( parentMap )
{
}

void DenseFrame::setPoints( const std::list< ColorPoint3d > &list )
{
    m_points = list;
}

const std::list< ColorPoint3d > &DenseFrame::points() const
{
    return m_points;
}

void DenseFrame::createOptimizationGrid()
{
    m_optimizationGrid.clear();

    for ( auto j : m_points ) {
        auto point = j.point();
        m_optimizationGrid[ point.x * 10 ][ point.y * 10 ][ point.z * 10 ].push_back( j );

    }

}

void DenseFrame::setOptimizationGrid( const OptimizationGrid &grid )
{
    m_optimizationGrid = grid;
}

// ProcessedDenseFrame
ProcessedDenseFrame::ProcessedDenseFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap ), ProcessedStereoKeyFrame( parentMap ), DenseFrame( parentMap )
{
}

void ProcessedDenseFrame::processDenseCloud()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {
        setPoints( this->parentMap()->parentWorld()->stereoProcessor().processPointList( leftFrame->image(), rightFrame->image() ) );

        // createOptimizationGrid();

    }

}

// FlowDenseFrame
FlowDenseFrame::FlowDenseFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap ), ProcessedStereoKeyFrame( parentMap ), FlowStereoKeyFrame( parentMap ), ProcessedDenseFrame( parentMap )
{
}

FlowDenseFrame::ObjectPtr FlowDenseFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FlowDenseFrame( parentMap ) );
}

// ConsecutiveFrame
ConsecutiveFrame::ConsecutiveFrame( const MonoFramePtr &startFrame, const MonoFramePtr &endFrame)
{
    setFrames( startFrame, endFrame );
}

MonoFramePtr ConsecutiveFrame::startFrame() const
{
    return std::dynamic_pointer_cast< MonoFrame >( frame1() );
}

MonoFramePtr ConsecutiveFrame::endFrame() const
{
    return std::dynamic_pointer_cast< MonoFrame >( frame2() );
}

std::vector< ConsecutivePoint > ConsecutiveFrame::points() const
{
    std::vector< ConsecutivePoint > ret;

    auto startFrame = this->startFrame();
    auto endFrame = this->endFrame();

    if ( startFrame && endFrame ) {

        auto points = startFrame->framePoints();

        for ( auto &i : points ) {

            if ( i ) {

                for ( auto j = i->nextPoint(); j; j = j->nextPoint() ) {

                    if ( j->parentFrame() == endFrame ) {

                        ret.push_back( ConsecutivePoint( i, j ) );
                        break;

                    }

                }

            }

        }

    }

    return ret;
}

std::vector< ConsecutivePoint > ConsecutiveFrame::mapPoints() const
{
    std::vector< ConsecutivePoint > ret;

    auto startFrame = this->startFrame();
    auto endFrame = this->endFrame();

    if ( startFrame && endFrame ) {

        auto points = startFrame->framePoints();

        for ( auto &i : points ) {

            if ( i && i->mapPoint() ) {

                for ( auto j = i->nextPoint(); j; j = j->nextPoint() ) {

                    if ( j->parentFrame() == endFrame ) {
                        ret.push_back( ConsecutivePoint( i, j ) );
                        break;

                    }

                }

            }

        }

    }

    return ret;
}

double ConsecutiveFrame::averageMapPointsDisplacement() const
{
    auto points = mapPoints();
    double disp = 0.;

    for ( auto &i : points ) {
        disp += cv::norm( i.startPoint() - i.endPoint() );
    }

    return disp / static_cast< double >( points.size() );
}

// FlowConsecutiveFrame
FlowConsecutiveFrame::FlowConsecutiveFrame( const FlowFramePtr &startFrame, const FlowFramePtr &endFrame )
    : ConsecutiveFrame( startFrame, endFrame )
{
}

FlowFramePtr FlowConsecutiveFrame::startFrame() const
{
    return std::dynamic_pointer_cast< FlowFrame >( ConsecutiveFrame::startFrame() );
}

FlowFramePtr FlowConsecutiveFrame::endFrame() const
{
    return std::dynamic_pointer_cast< FlowFrame >( ConsecutiveFrame::endFrame() );
}

// RecoverPoseFrame
RecoverPoseFrame::RecoverPoseFrame( const ProcessedKeyFramePtr &startFrame, const ProcessedFramePtr &endFrame)
    : ConsecutiveFrame( startFrame, endFrame )
{
}

ProcessedKeyFramePtr RecoverPoseFrame::startFrame() const
{
    return std::dynamic_pointer_cast< ProcessedKeyFrame >( ConsecutiveFrame::startFrame() );
}

ProcessedFramePtr RecoverPoseFrame::endFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( ConsecutiveFrame::endFrame() );
}

double RecoverPoseFrame::recoverPose( ProjectionMatrix *result )
{
    if ( result ) {

        std::vector< cv::Point3f > points3d;
        std::vector< cv::Point2f > points2d;

        auto startFrame = this->startFrame();
        auto endFrame = this->endFrame();

        if ( startFrame && endFrame ) {

            auto parentWorld = startFrame->parentWorld();

            auto maxReprojectionError = parentWorld->maxReprojectionError();

            auto points = mapPoints();

            for ( auto &i : points ) {

                auto mapPoint = i.endFramePoint()->mapPoint();

                if ( mapPoint ) {

                    points3d.push_back( mapPoint->point() );
                    points2d.push_back( i.endPoint() );

                }

            }

            if ( points3d.size() < MIN_PNP_POINTS_COUNT )
                return 0.;

            cv::Mat rvec;
            cv::Mat tvec;

            std::vector< int > inliers;

            if ( !cv::solvePnPRansac( points3d, points2d, startFrame->cameraMatrix(), cv::noArray(), rvec, tvec, false,
                                                    500, maxReprojectionError, 1.0 - 1.e-5, inliers, cv::SOLVEPNP_ITERATIVE ) )
                return 0.;

            std::set< int > inliersSet;

            for ( auto i : inliers )
                inliersSet.insert( i );

            for ( size_t i = 0; i < points.size(); ++i )

                if ( inliersSet.find( i ) == inliersSet.end() ) {

                    auto point = points[ i ].endFramePoint();

                    auto prevPoint = point->prevPoint();

                    if ( prevPoint )
                        prevPoint->clearNextPoint();

                    point->clearPrevPoint();
                    point->clearMapPoint();

                }

            cv::Mat rmat;
            cv::Rodrigues( rvec, rmat );

            result->setCameraMatrix( startFrame->cameraMatrix() );
            result->setTranslation( tvec );
            result->setRotation( rmat );

            return static_cast< double >( inliers.size() ) / points.size();

        }

    }

    return 0.;
}

// ConsecutiveKeyFrame
ConsecutiveKeyFrame::ConsecutiveKeyFrame(const ProcessedKeyFramePtr &startFrame, const ProcessedKeyFramePtr &endFrame )
    : ConsecutiveFrame( startFrame, endFrame )
{
}

ProcessedKeyFramePtr ConsecutiveKeyFrame::startFrame() const
{
    return std::dynamic_pointer_cast< ProcessedKeyFrame >( ConsecutiveFrame::startFrame() );
}

ProcessedKeyFramePtr ConsecutiveKeyFrame::endFrame() const
{
    return std::dynamic_pointer_cast< ProcessedKeyFrame >( ConsecutiveFrame::endFrame() );
}

int ConsecutiveKeyFrame::triangulatePoints()
{
    int ret = 0;

    auto startFrame = this->startFrame();
    auto endFrame = this->endFrame();

    if ( startFrame && endFrame ) {

        auto maxReprojectionError = startFrame->parentWorld()->maxReprojectionError();
        auto minAdjacentPointsDistance = startFrame->parentWorld()->minAdjacentPointsDistance();

        auto points = this->points();

        if ( !points.empty() ) {

            cv::Mat_< float > startPoints( 2, points.size() ),
                              endPoints( 2, points.size() );

            for ( size_t i = 0; i < points.size(); ++i ) {

                auto startPoint = points[i].startPoint();
                startPoints.row( 0 ).col( i ) = startPoint.x;
                startPoints.row( 1 ).col( i ) = startPoint.y;

                auto endPoint = points[ i ].endPoint();
                endPoints.row( 0 ).col( i ) = endPoint.x;
                endPoints.row( 1 ).col( i ) = endPoint.y;

            }

            cv::Mat points3d;

            auto startProjectionMatrix = startFrame->projectionMatrix();
            auto endProjectionMatrix = endFrame->projectionMatrix();

            cv::triangulatePoints( startProjectionMatrix, endProjectionMatrix, startPoints, endPoints, points3d );

            for ( size_t i = 0; i < points.size(); ++i ) {

                if ( points[ i ].distance() > minAdjacentPointsDistance ) {

                    auto w = points3d.at< float >( 3, i );

                    if ( std::abs( w ) > FLOAT_EPS ) {

                        auto x = points3d.at< float >( 0, i ) / w;
                        auto y = points3d.at< float >( 1, i ) / w;
                        auto z = points3d.at< float >( 2, i ) / w;

                        auto pt = cv::Point3d( x, y, z );

                        cv::Mat pt4d( 4, 1, CV_64F );
                        points3d.col( i ).convertTo( pt4d, CV_64F );

                        cv::Mat startReprojMat = startProjectionMatrix * pt4d;
                        cv::Mat endReprojMat = endProjectionMatrix * pt4d;

                        auto startW = startReprojMat.at< double >( 2, 0 );
                        auto endW = endReprojMat.at< double >( 2, 0 );

                        if ( std::abs( startW ) > DOUBLE_EPS && std::abs( endW ) > DOUBLE_EPS &&
                                    startW / w > 0 && endW / w > 0 ) {

                            cv::Point2d startReprojPt( startReprojMat.at< double >( 0, 0 ) / startW,
                                                      startReprojMat.at< double >( 1, 0 ) / startW );

                            cv::Point2d endReprojPt( endReprojMat.at< double >( 0, 0 ) / endW,
                                                      endReprojMat.at< double >( 1, 0 ) / endW );

                            auto startNorm = cv::norm( startReprojPt - cv::Point2d( points[ i ].startPoint() ) );
                            auto endNorm = cv::norm( endReprojPt - cv::Point2d( points[ i ].endPoint() ) );

                            if ( startNorm < maxReprojectionError && endNorm < maxReprojectionError ) {

                                ++ret;

                                auto startFramePoint = points[ i ].startFramePoint();
                                auto endFramePoint = points[ i ].endFramePoint();

                                auto mapPoint = startFramePoint->mapPoint();

                                if ( !mapPoint ) {
                                    mapPoint = endFrame->parentMap()->createMapPoint( pt, startFramePoint->color() );
                                }
                                else {
                                    mapPoint->setPoint( pt );
                                }

                                for ( auto j = endFramePoint; j; j = j->prevPoint() ) {

                                    j->setMapPoint( mapPoint );

                                    auto stereoPoint = j->stereoPoint();

                                    if ( stereoPoint )
                                        stereoPoint->setMapPoint( mapPoint );

                                }

                            }

                        }

                    }

                }

            }

        }

    }

    return ret;

}

double ConsecutiveKeyFrame::distance() const
{
    return cv::norm( endFrame()->translationVector() - startFrame()->translationVector() );
}

// FinishedStereoFrame
FinishedStereoFrame::FinishedStereoFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap )
{
}

FinishedStereoFrame::ObjectPtr FinishedStereoFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FinishedStereoFrame( parentMap ) );
}

void FinishedStereoFrame::setFrames( const FinishedFramePtr &leftFrame, const FinishedFramePtr &rightFrame )
{
    DoubleFrame::setFrames( leftFrame, rightFrame );
}

FinishedFramePtr FinishedStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FinishedFrame >( StereoFrame::leftFrame() );
}

FinishedFramePtr FinishedStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FinishedFrame >( StereoFrame::rightFrame() );
}

void FinishedStereoFrame::replace( const ProcessedStereoFramePtr &frame )
{
    if ( frame ) {

        auto leftFrame = this->leftFrame();
        if ( !leftFrame ) {
            leftFrame = FinishedFrame::create();
            setLeftFrame( leftFrame );
        }

        auto rightFrame = this->rightFrame();
        if ( !rightFrame ) {
            rightFrame = FinishedFrame::create();
            setRightFrame( rightFrame );
        }

        auto leftProcessedFrame = frame->leftFrame();
        auto rightProcessedFrame = frame->rightFrame();

        if ( leftFrame && leftProcessedFrame )
            leftFrame->replace( leftProcessedFrame );

        if ( rightFrame && rightProcessedFrame )
            rightFrame->replace( rightProcessedFrame );

    }

}

void FinishedStereoFrame::replaceAndClean( const ProcessedStereoFramePtr &frame )
{
    if ( frame ) {

        auto leftFrame = this->leftFrame();
        if ( !leftFrame ) {
            leftFrame = FinishedFrame::create();
            setLeftFrame( leftFrame );
        }

        auto rightFrame = this->rightFrame();
        if ( !rightFrame ) {
            rightFrame = FinishedFrame::create();
            setRightFrame( rightFrame );
        }

        auto leftProcessedFrame = frame->leftFrame();
        auto rightProcessedFrame = frame->rightFrame();

        if ( leftFrame && leftProcessedFrame )
            leftFrame->replaceAndClean( leftProcessedFrame );

        if ( rightFrame && rightProcessedFrame )
            rightFrame->replaceAndClean( rightProcessedFrame );

    }

}

// FinishedStereoKeyFrame
FinishedStereoKeyFrame::FinishedStereoKeyFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), StereoKeyFrame( parentMap )
{
}

FinishedStereoKeyFrame::ObjectPtr FinishedStereoKeyFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FinishedStereoKeyFrame( parentMap ) );
}

void FinishedStereoKeyFrame::setFrames( const FinishedKeyFramePtr &leftFrame, const FinishedKeyFramePtr &rightFrame )
{
    DoubleFrame::setFrames( leftFrame, rightFrame );
}

FinishedKeyFramePtr FinishedStereoKeyFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FinishedKeyFrame >( StereoFrame::leftFrame() );
}

FinishedKeyFramePtr FinishedStereoKeyFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FinishedKeyFrame >( StereoFrame::rightFrame() );
}

void FinishedStereoKeyFrame::replace( const ProcessedStereoKeyFramePtr &frame )
{
    if ( frame ) {

        auto leftFrame = this->leftFrame();
        if ( !leftFrame ) {
            leftFrame = FinishedKeyFrame::create();
            setLeftFrame( leftFrame );
        }

        auto rightFrame = this->rightFrame();
        if ( !rightFrame ) {
            rightFrame = FinishedKeyFrame::create();
            setRightFrame( rightFrame );
        }

        auto leftProcessedFrame = frame->leftFrame();
        auto rightProcessedFrame = frame->rightFrame();

        if ( leftFrame && leftProcessedFrame )
            leftFrame->replace( leftProcessedFrame );

        if ( rightFrame && rightProcessedFrame )
            rightFrame->replace( rightProcessedFrame );

    }

}

void FinishedStereoKeyFrame::replaceAndClean( const ProcessedStereoKeyFramePtr &frame )
{
    if ( frame ) {

        auto leftFrame = this->leftFrame();
        if ( !leftFrame ) {
            leftFrame = FinishedKeyFrame::create();
            setLeftFrame( leftFrame );
        }

        auto rightFrame = this->rightFrame();
        if ( !rightFrame ) {
            rightFrame = FinishedKeyFrame::create();
            setRightFrame( rightFrame );
        }

        auto leftProcessedFrame = frame->leftFrame();
        auto rightProcessedFrame = frame->rightFrame();

        if ( leftFrame && leftProcessedFrame )
            leftFrame->replaceAndClean( leftProcessedFrame );

        if ( rightFrame && rightProcessedFrame )
            rightFrame->replaceAndClean( rightProcessedFrame );

    }

}

// FinishedDenseFrame
const double FinishedDenseFrame::m_maximumLenght = 40;

FinishedDenseFrame::FinishedDenseFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), StereoKeyFrame( parentMap ), FinishedStereoKeyFrame( parentMap ), DenseFrame( parentMap )
{
}

FinishedDenseFrame::ObjectPtr FinishedDenseFrame::create( const MapPtr &parentMap )
{
    return ObjectPtr( new FinishedDenseFrame( parentMap ) );
}

std::list< ColorPoint3d > FinishedDenseFrame::translatedPoints() const
{
    std::list< ColorPoint3d > ret;

    auto leftFrame = this->leftFrame();

    if ( leftFrame ) {
        cv::Mat rotation = leftFrame->rotation().t();
        cv::Mat translation = leftFrame->translation();

        for ( auto &i : m_points ) {
            auto point = i;
            point.setPoint( matToPoint3f< double >( rotation * ( point3fToMat< double >( i.point() ) - translation ) ) );
            ret.push_back( point );

        }

    }

    return ret;

}

void FinishedDenseFrame::replace( const ProcessedDenseFramePtr &frame )
{
    FinishedStereoKeyFrame::replace( frame );

    replaceProcedure( frame );

}

void FinishedDenseFrame::replaceAndClean( const ProcessedDenseFramePtr &frame )
{
    FinishedStereoKeyFrame::replaceAndClean( frame );

    replaceProcedure( frame );

}

void FinishedDenseFrame::replaceProcedure( const ProcessedDenseFramePtr &frame )
{
    if ( frame ) {

        // TODO: Smarter selection

        for ( auto &i : frame->m_points )
            if ( cv::norm( i.point() ) < m_maximumLenght )
                m_points.push_back( i );

        setOptimizationGrid( frame->m_optimizationGrid );

    }

}

}

