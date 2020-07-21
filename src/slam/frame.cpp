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
const double ProcessedFrame::m_minPointsDistance = 10.0;

ProcessedFrame::ProcessedFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
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

// ProcessedKeyFrame
ProcessedKeyFrame::ProcessedKeyFrame( const MapPtr &parentMap )
    : ProcessedFrame( parentMap )
{
}

// FlowFrame
FlowFrame::FlowFrame( const FlowMapPtr &parentMap)
    : ProcessedFrame( parentMap )
{
    initialize();
}

void FlowFrame::initialize()
{
}

FlowFrame::ObjectPtr FlowFrame::create( const FlowMapPtr &parentMap )
{
    return ObjectPtr( new FlowFrame( parentMap ) );
}

void FlowFrame::setImage( const StampedImage &image )
{
    ProcessedFrame::setImage( image );

    m_searchMatrix.create( image.rows, image.cols );
}

void FlowFrame::createMask()
{
    if ( !m_image.empty() ) {
        m_mask = cv::Mat( m_image.rows, m_image.cols, CV_8U, cv::Scalar( 1 ) );

        auto distance = parentWorld()->flowTracker()->minDistance();

        for ( auto &i : m_points ) {
            if ( i ) {
                auto pt = i->point();
                cv::rectangle( m_mask, cv::Rect( pt.x - distance, pt.y - distance, distance * 2, distance * 2 ), cv::Scalar( 0 ) );
            }

        }

    }
    else
        m_mask.release();
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
    return std::vector< MonoPointPtr >( m_points.begin(), m_points.end() );
}

void FlowFrame::removePoint( const MonoPointPtr &point )
{
    if ( point ) {

        m_points.erase( std::dynamic_pointer_cast< FlowPoint >( point ) );

        auto pt =  point->point();

        m_searchMatrix.at( pt.y, pt.x ).reset();

    }

}

std::vector< FlowPointPtr > FlowFrame::flowPoints() const
{
    return std::vector< FlowPointPtr >( m_points.begin(), m_points.end() );
}

FlowPointPtr FlowFrame::framePoint( const cv::Point2f &point ) const
{
    return m_searchMatrix.at( point.y, point.x );
}

FlowPointPtr FlowFrame::addFramePoint( const cv::Point2f &point )
{
    auto searchPoint = framePoint( point );

    if ( searchPoint )
        return searchPoint;

    auto flowPoint = FlowPoint::create( shared_from_this(), point, m_image.at< cv::Vec3b >( point ) );

    flowPoint->setError( 0. );

    m_points.insert( flowPoint );

    m_searchMatrix.at( point.y, point.x ) = flowPoint;

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

        std::vector< PointTrackResult > trackedSet;

        auto fmat = prevFrame->parentWorld()->flowTracker()->track( prevFrame, nextFrame, &trackedSet );

        auto trackPoints = prevFrame->flowPoints();

        for ( auto &i : trackedSet ) {

            auto prevPoint = trackPoints[ i.index ];
            auto nextPoint = nextFrame->addFramePoint( i );

            prevPoint->setNextPoint( nextPoint );
            nextPoint->setPrevPoint( prevPoint );

            auto worlPoint = prevPoint->mapPoint();

            if ( worlPoint )
                nextPoint->setMapPoint( worlPoint );

            nextPoint->setError( prevPoint->error() + i.error );

        }

        return fmat;

    }

    return cv::Mat();

}

// FlowKeyFrame
FlowKeyFrame::FlowKeyFrame( const FlowMapPtr &parentMap )
    : ProcessedFrame( parentMap ), FlowFrame( parentMap ), ProcessedKeyFrame( parentMap )
{
    initialize();
}

void FlowKeyFrame::initialize()
{
    m_usePointsCount = 0;
}

FlowKeyFrame::ObjectPtr FlowKeyFrame::create( const FlowMapPtr &parentMap )
{
    return ObjectPtr( new FlowKeyFrame( parentMap ) );
}

std::vector< cv::Point2f > FlowKeyFrame::extractedPoints() const
{
    return m_extractedPoints;
}

size_t FlowKeyFrame::extractedPointsCount() const
{
    return m_extractedPoints.size();
}

void FlowKeyFrame::setImage( const StampedImage &image )
{
    FlowFrame::setImage( image );

    m_extractedPoints.clear();
    m_usePointsCount = 0;
}

void FlowKeyFrame::extractPoints()
{
    createMask();

    parentWorld()->flowTracker()->extractPoints( this );
}

CvImage FlowKeyFrame::drawExtractedPoints() const
{
    if ( !m_image.empty() ) {

        int radius = std::max( std::min( m_image.width(), m_image.height() ) / 300.0, 1.0 );

        CvImage ret;
        m_image.copyTo( ret );

        auto points = extractedPoints();

        drawFeaturePoints( &ret, points, radius );

        drawLabel( &ret, "Extracted points count: " + std::to_string( points.size() ), ret.height() / 70 );

        return ret;

    }

    return CvImage();

}

void FlowKeyFrame::setExtractedPoints( const std::vector< cv::Point2f > &value )
{
    m_usePointsCount = 0;

    m_extractedPoints = value;
}

size_t FlowKeyFrame::usePointsCount() const
{
    return m_usePointsCount;
}

void FlowKeyFrame::createFramePoints( const size_t count )
{
    if ( m_extractedPoints.empty() )
        extractPoints();

    for ( size_t i = 0; i < count; ++i ) {

        if ( m_usePointsCount >= m_extractedPoints.size() )
            return;

        createFramePoint( m_usePointsCount );

        ++m_usePointsCount;
    }

}

void FlowKeyFrame::replace( const FlowFramePtr &frame )
{
    if ( frame ) {

        setImage( frame->image() );
        setImagePyramid( frame->imagePyramid() );

        m_points = frame->m_points;

        frame->m_points.clear();

        for ( auto &i : m_points )
            i->setParentFrame( shared_from_this() );

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

FlowPointPtr FlowKeyFrame::createFramePoint( const size_t keyPointIndex )
{
    if ( keyPointIndex >= m_extractedPoints.size() )
        return FlowPointPtr();

    auto ret = addFramePoint( m_extractedPoints[ keyPointIndex ] );

    return ret;
}

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

std::vector< cv::Point2f > FeatureFrame::extractedPoints() const
{
    std::vector< cv::Point2f > ret;
    ret.reserve( m_keyPoints.size() );

    for ( auto &i : m_keyPoints )
        ret.push_back( i.pt );

    return ret;
}

size_t FeatureFrame::extractedPointsCount() const
{
    return m_keyPoints.size();
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
    createMask();

    parentWorld()->featureTracker()->extractKeypoints( this );

    for ( size_t i = 0; i < m_keyPoints.size(); ++i )
        m_colors.push_back( m_image.at< cv::Vec3b >( m_keyPoints[ i ].pt ) );

}

CvImage FeatureFrame::drawExtractedKeypoints() const
{
    if ( !m_image.empty() ) {

        int radius = std::max( std::min( m_image.width(), m_image.height() ) / 300.0, 1.0 );

        CvImage ret;
        m_image.copyTo( ret );

        auto points = extractedPoints();

        drawFeaturePoints( &ret, points, radius );

        drawLabel( &ret, "Extracted points count: " + std::to_string( points.size() ), ret.height() / 70 );

        return ret;

    }

    return CvImage();

}

void FeatureFrame::createMask()
{
    if ( !m_image.empty() ) {
        m_mask = cv::Mat( m_image.rows, m_image.cols, CV_8U, cv::Scalar( 1 ) );

        auto distance = parentWorld()->pointsMinDistance();

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

// FeatureKeyFrame
FeatureKeyFrame::FeatureKeyFrame( const FeatureMapPtr &parentMap )
    : ProcessedFrame( parentMap ), FeatureFrame( parentMap ), ProcessedKeyFrame( parentMap )
{
}

FeatureKeyFrame::ObjectPtr FeatureKeyFrame::create( const FeatureMapPtr &parentMap )
{
    return ObjectPtr( new FeatureKeyFrame( parentMap ) );
}

// FinishedFrame
FinishedFrame::FinishedFrame( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    setTime( time );
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

std::shared_ptr< FinishedFrame > FinishedFrame::shared_from_this()
{
    return std::dynamic_pointer_cast< FinishedFrame >( MonoFrame::shared_from_this() );
}

std::shared_ptr< const FinishedFrame > FinishedFrame::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FinishedFrame >( MonoFrame::shared_from_this() );
}

FramePointPtr FinishedFrame::createFramePoint( const cv::Point2f &point, const cv::Scalar &color )
{
    auto ret = FramePoint::create( shared_from_this(), point, color );

    m_points.push_back( ret );

    return ret;

}

FinishedFrame::ObjectPtr FinishedFrame::create( const std::chrono::time_point< std::chrono::system_clock > time )
{
    return ObjectPtr( new FinishedFrame( time ) );
}


void FinishedFrame::replace( const ProcessedKeyFramePtr &frame )
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

void FinishedFrame::replaceAndClean( const ProcessedKeyFramePtr &frame )
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
    return std::dynamic_pointer_cast< ProcessedFrame >( frame1() );
}

ProcessedFramePtr ProcessedStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( frame2() );
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
    return std::dynamic_pointer_cast< ProcessedKeyFrame >( ProcessedStereoFrame::leftFrame() );
}

ProcessedKeyFramePtr ProcessedStereoKeyFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcessedKeyFrame >( ProcessedStereoFrame::rightFrame() );
}

// FlowStereoFrame
FlowStereoFrame::FlowStereoFrame( const FlowMapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap )
{
    setFrames( FlowFrame::create( parentMap ), FlowFrame::create( parentMap ) );
}

FlowStereoFrame::ObjectPtr FlowStereoFrame::create( const FlowMapPtr &parentMap )
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
    return std::dynamic_pointer_cast< FlowFrame >( ProcessedStereoFrame::leftFrame() );
}

FlowFramePtr FlowStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FlowFrame >( ProcessedStereoFrame::rightFrame() );
}

FlowMapPtr FlowStereoFrame::parentMap() const
{
    return std::dynamic_pointer_cast< FlowMap >( ProcessedStereoFrame::parentMap() );
}

cv::Mat FlowStereoFrame::match()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        std::vector< PointTrackResult > trackedSet;

        auto fmat = parentWorld()->flowTracker()->track( leftFrame, rightFrame, &trackedSet );

        auto trackPoints = leftFrame->flowPoints();

        auto minDistance = parentWorld()->minStereoDisparity();

        for ( auto &i : trackedSet ) {

            auto leftFlowPoint = trackPoints[ i.index ];
            auto leftPoint = leftFlowPoint->point();
            auto rightPoint = i;

            if ( cv::norm( leftPoint - rightPoint ) > minDistance ) {

                auto rightFlowPoint = rightFrame->addFramePoint( rightPoint );

                leftFlowPoint->setStereoPoint( rightFlowPoint );
                rightFlowPoint->setStereoPoint( leftFlowPoint );

                auto worlPoint = leftFlowPoint->mapPoint();

                if ( worlPoint )
                    rightFlowPoint->setMapPoint( worlPoint );

                rightFlowPoint->setError( leftFlowPoint->error() + i.error );

            }

        }

        return fmat;

    }

    return cv::Mat();

}

// FlowStereoKeyFrame
FlowStereoKeyFrame::FlowStereoKeyFrame( const FlowMapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap ), ProcessedStereoKeyFrame( parentMap ), FlowStereoFrame( parentMap )
{
    setFrames( FlowKeyFrame::create( parentMap ), FlowKeyFrame::create( parentMap ) );
}

FlowStereoKeyFrame::ObjectPtr FlowStereoKeyFrame::create( const FlowMapPtr &parentMap )
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
    return std::dynamic_pointer_cast< FlowKeyFrame >( ProcessedStereoFrame::leftFrame() );
}

FlowKeyFramePtr FlowStereoKeyFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FlowKeyFrame >( ProcessedStereoFrame::rightFrame() );
}

CvImage FlowStereoKeyFrame::drawExtractedPoints() const
{
    CvImage leftImage;

    auto leftFeatureFrame = this->leftFrame();

    if ( leftFeatureFrame )
        leftImage = leftFeatureFrame->drawExtractedPoints();

    return leftImage;
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

// FeatureStereoFrame
FeatureStereoFrame::FeatureStereoFrame( const FeatureMapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap )
{
    setFrames( FeatureFrame::create( parentMap ), FeatureFrame::create( parentMap ) );
}

FeatureStereoFrame::ObjectPtr FeatureStereoFrame::create( const FeatureMapPtr &parentMap )
{
    return ObjectPtr( new FeatureStereoFrame( parentMap ) );
}

void FeatureStereoFrame::loadLeft( const StampedImage &image )
{
    auto frame = leftFrame();

    if ( frame )
        frame->setImage( image );
}

void FeatureStereoFrame::loadRight( const StampedImage &image )
{
    auto frame = rightFrame();

    if ( frame )
        frame->setImage( image );
}

void FeatureStereoFrame::load( const StampedImage &leftImage, const StampedImage &rightImage )
{
    loadLeft( leftImage );
    loadRight( rightImage );
}

FeatureFramePtr FeatureStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( ProcessedStereoFrame::leftFrame() );
}

FeatureFramePtr FeatureStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( ProcessedStereoFrame::rightFrame() );
}

FeatureMapPtr FeatureStereoFrame::parentMap() const
{
    return std::dynamic_pointer_cast< FeatureMap >( ProcessedStereoFrame::parentMap() );
}

cv::Mat FeatureStereoFrame::match()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        std::vector< cv::DMatch > matches;

        auto fmat = parentWorld()->featureTracker()->match( leftFrame, rightFrame, &matches );

        auto trackPoints = leftFrame->featurePoints();

        auto minDistance = parentWorld()->minAdjacentPointsDistance();

        for ( auto &i : matches ) {

            auto leftPoint = trackPoints[ i.queryIdx ];
            auto rightPoint = rightFrame->featurePoint( i.trainIdx );

            if ( cv::norm( leftPoint->point() - rightPoint->point() ) > minDistance ) {

                leftPoint->setStereoPoint( rightPoint );
                rightPoint->setStereoPoint( leftPoint );

                auto worlPoint = leftPoint->mapPoint();

                if ( worlPoint )
                    rightPoint->setMapPoint( worlPoint );

            }

        }

        return fmat;

    }

    return cv::Mat();

}

// FeatureStereoKeyFrame
FeatureStereoKeyFrame::FeatureStereoKeyFrame( const FeatureMapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap ), ProcessedStereoKeyFrame( parentMap ), FeatureStereoFrame( parentMap )
{
    setFrames( FeatureKeyFrame::create( parentMap ), FeatureKeyFrame::create( parentMap ) );
}

FeatureStereoKeyFrame::ObjectPtr FeatureStereoKeyFrame::create( const FeatureMapPtr &parentMap )
{
    return ObjectPtr( new FeatureStereoKeyFrame( parentMap ) );
}

void FeatureStereoKeyFrame::extractKeypoints()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame )
        leftFrame->extractKeypoints();

    if ( rightFrame )
        rightFrame->extractKeypoints();

}

FeatureKeyFramePtr FeatureStereoKeyFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FeatureKeyFrame >( ProcessedStereoFrame::leftFrame() );
}

FeatureKeyFramePtr FeatureStereoKeyFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FeatureKeyFrame >( ProcessedStereoFrame::rightFrame() );
}

CvImage FeatureStereoKeyFrame::drawExtractedPoints() const
{
    CvImage leftImage;

    auto leftFrame = this->leftFrame();

    if ( leftFrame )
        leftImage = leftFrame->drawExtractedKeypoints();

    return leftImage;
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
FlowDenseFrame::FlowDenseFrame( const FlowMapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap ), ProcessedStereoKeyFrame( parentMap ), FlowStereoKeyFrame( parentMap ), ProcessedDenseFrame( parentMap )
{

}

FlowDenseFrame::ObjectPtr FlowDenseFrame::create( const FlowMapPtr &parentMap )
{
    return ObjectPtr( new FlowDenseFrame( parentMap ) );
}

// FeatureDenseFrame
FeatureDenseFrame::FeatureDenseFrame( const FeatureMapPtr &parentMap )
    : StereoFrame( parentMap ), ProcessedStereoFrame( parentMap ), StereoKeyFrame( parentMap ), ProcessedStereoKeyFrame( parentMap ), FeatureStereoKeyFrame( parentMap ), ProcessedDenseFrame( parentMap )
{
}

FeatureDenseFrame::ObjectPtr FeatureDenseFrame::create( const FeatureMapPtr &parentMap )
{
    return ObjectPtr( new FeatureDenseFrame( parentMap ) );
}

// ConsecutiveFrame
ConsecutiveFrame::ConsecutiveFrame( const MonoFramePtr &startFrame, const MonoFramePtr &endFrame, const MapPtr &parentMap )
    : m_parentMap( parentMap )
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

MapPtr ConsecutiveFrame::parentMap() const
{
    return m_parentMap.lock();
}

WorldPtr ConsecutiveFrame::parentWorld() const
{
    return parentMap()->parentWorld();
}

// FlowConsecutiveFrame
FlowConsecutiveFrame::FlowConsecutiveFrame( const FlowFramePtr &startFrame, const FlowFramePtr &endFrame , const MapPtr &parentMap )
    : ConsecutiveFrame( startFrame, endFrame, parentMap )
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

// FeatureConsecutiveFrame
FeatureConsecutiveFrame::FeatureConsecutiveFrame( const FeatureFramePtr &startFrame, const FeatureFramePtr &endFrame , const MapPtr &parentMap )
    : ConsecutiveFrame( startFrame, endFrame, parentMap )
{
}

FeatureFramePtr FeatureConsecutiveFrame::startFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( ConsecutiveFrame::startFrame() );
}

FeatureFramePtr FeatureConsecutiveFrame::endFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( ConsecutiveFrame::endFrame() );
}

// RecoverPoseFrame
RecoverPoseFrame::RecoverPoseFrame( const MonoKeyFramePtr &startFrame, const MonoFramePtr &endFrame, const MapPtr &parentMap )
    : ConsecutiveFrame( startFrame, endFrame, parentMap )
{
}

MonoKeyFramePtr RecoverPoseFrame::startFrame() const
{
    return std::dynamic_pointer_cast< MonoKeyFrame >( ConsecutiveFrame::startFrame() );
}

MonoFramePtr RecoverPoseFrame::endFrame() const
{
    return std::dynamic_pointer_cast< MonoFrame >( ConsecutiveFrame::endFrame() );
}

double RecoverPoseFrame::recoverPose( ProjectionMatrix *result )
{
    if ( result ) {

        std::vector< cv::Point3f > points3d;
        std::vector< cv::Point2f > points2d;

        auto startFrame = this->startFrame();
        auto endFrame = this->endFrame();

        if ( startFrame && endFrame ) {

            auto parentWorld = this->parentWorld();

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

// TriangulateFrame
TriangulateFrame::TriangulateFrame( const MonoKeyFramePtr &startFrame, const MonoKeyFramePtr &endFrame, const MapPtr &parentMap )
    : ConsecutiveFrame( startFrame, endFrame, parentMap )
{
}

MonoKeyFramePtr TriangulateFrame::startFrame() const
{
    return std::dynamic_pointer_cast< MonoKeyFrame >( ConsecutiveFrame::startFrame() );
}

MonoKeyFramePtr TriangulateFrame::endFrame() const
{
    return std::dynamic_pointer_cast< MonoKeyFrame >( ConsecutiveFrame::endFrame() );
}

int TriangulateFrame::triangulatePoints()
{
    int ret = 0;

    auto maxReprojectionError = parentWorld()->maxReprojectionError();

    auto points = this->points();

    for ( auto &i : points ) {

    }

/*
    std::map< FramePtr, std::vector< PointPtr > > frames;
    std::map< PointPtr, PointPtr > points;

    auto framePoints = this->framePoints();

    auto maxReprojectionError = parentWorld()->maxReprojectionError();

    for ( auto &i : framePoints ) {

        if ( i ) {

            auto prevPtr = i->prevPoint();

            if ( prevPtr ) {

                PointPtr j;

                for ( j = prevPtr; j->prevPoint(); j = j->prevPoint() );

                if ( j ) {

                    auto parentFrame = j->parentFrame();

                    if ( parentFrame ) {

                        auto cameraDistance = cv::norm( static_cast< cv::Point3d >( *this ) - static_cast< cv::Point3d >( *parentFrame ) );
                        auto pointsDistance = cv::norm( j->point() - i->point() );

                        if ( cameraDistance > parentMap()->minTriangulateCameraDistance() && pointsDistance > m_minPointsDistance ) {
                            frames[ parentFrame ].push_back( j );
                            points[ j ] = i;

                        }

                    }

                }

            }

        }

    }

    for ( auto &i : frames ) {

        cv::Mat points3d;

        cv::Mat_< float > prevPoints( 2, i.second.size() ),
                          nextPoints( 2, i.second.size() );

        for ( size_t j = 0; j < i.second.size(); ++j ) {
            auto sourcePoint = i.second[ j ];

            auto prevPoint = sourcePoint->point();
            prevPoints.row( 0 ).col( j ) = prevPoint.x;
            prevPoints.row( 1 ).col( j ) = prevPoint.y;

            auto nextPoint = points[ sourcePoint ]->point();
            nextPoints.row( 0 ).col( j ) = nextPoint.x;
            nextPoints.row( 1 ).col( j ) = nextPoint.y;

        }

        if ( !prevPoints.empty() && prevPoints.size() == nextPoints.size() ) {

            auto prevProjectionMatrix = i.first->projectionMatrix();
            auto nextProjectionMatrix = projectionMatrix();

            cv::triangulatePoints( prevProjectionMatrix, nextProjectionMatrix, prevPoints, nextPoints, points3d );

            for ( size_t j = 0; j < i.second.size(); ++j ) {

                auto sourcePoint = i.second[ j ];

                auto processedPoint = points[ sourcePoint ];

                if ( processedPoint ) {

                    auto w = points3d.at< float >( 3, j );

                    if ( std::abs( w ) > FLOAT_EPS ) {

                        auto x = points3d.at< float >( 0, j ) / w;
                        auto y = points3d.at< float >( 1, j ) / w;
                        auto z = points3d.at< float >( 2, j ) / w;

                        auto pt = cv::Point3d( x, y, z );

                        cv::Mat pt4d( 4, 1, CV_64F );
                        points3d.col( j ).convertTo( pt4d, CV_64F );

                        cv::Mat prevReprojMat = prevProjectionMatrix * pt4d;
                        cv::Mat nextReprojMat = nextProjectionMatrix * pt4d;

                        auto prevW = prevReprojMat.at< double >( 2, 0 );
                        auto nextW = nextReprojMat.at< double >( 2, 0 );

                        if ( std::abs( prevW ) > DOUBLE_EPS && std::abs( nextW ) > DOUBLE_EPS &&
                                    prevW / w > 0 && nextW / w > 0 ) {

                            cv::Point2d prevReprojPt( prevReprojMat.at< double >( 0, 0 ) / prevW,
                                                      prevReprojMat.at< double >( 1, 0 ) / prevW );

                            cv::Point2d nextReprojPt( nextReprojMat.at< double >( 0, 0 ) / nextW,
                                                      nextReprojMat.at< double >( 1, 0 ) / nextW );

                            auto prevNorm = cv::norm( prevReprojPt - cv::Point2d( sourcePoint->point() ) );
                            auto nextNorm = cv::norm( nextReprojPt - cv::Point2d( processedPoint->point() ) );

                            if ( prevNorm < maxReprojectionError && nextNorm < maxReprojectionError ) {

                                ++ret;

                                auto mapPoint = processedPoint->mapPoint();

                                if ( !mapPoint ) {
                                    mapPoint = parentMap()->createMapPoint( pt, sourcePoint->color() );
                                }
                                else {
                                    mapPoint->setPoint( pt );
                                    mapPoint->setColor( sourcePoint->color() );

                                }

                                for ( PointPtr k = processedPoint; k; k = k->prevPoint() ) {
                                    k->setMapPoint( mapPoint );
                                    auto stereoPoint = k->stereoPoint();
                                    if ( stereoPoint )
                                        stereoPoint->setMapPoint( mapPoint );

                                }

                            }

                        }

                    }

                }

            }

        }

    }*/

    return ret;

}

double TriangulateFrame::distance() const
{
    return cv::norm( endFrame()->translationVector() - startFrame()->translationVector() );
}

// FinishedStereoFrame
FinishedStereoFrame::FinishedStereoFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), StereoKeyFrame( parentMap )
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
    return std::dynamic_pointer_cast< FinishedFrame >( DoubleFrame::frame1() );
}

FinishedFramePtr FinishedStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FinishedFrame >( DoubleFrame::frame2() );
}

void FinishedStereoFrame::replace( const ProcessedStereoKeyFramePtr &frame )
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

void FinishedStereoFrame::replaceAndClean( const ProcessedStereoKeyFramePtr &frame )
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

// FinishedDenseFrame
const double FinishedDenseFrame::m_maximumLenght = 40;

FinishedDenseFrame::FinishedDenseFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap ), StereoKeyFrame( parentMap ), FinishedStereoFrame( parentMap ), DenseFrame( parentMap )
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
    FinishedStereoFrame::replace( frame );

    replaceProcedure( frame );

}

void FinishedDenseFrame::replaceAndClean( const ProcessedDenseFramePtr &frame )
{
    FinishedStereoFrame::replaceAndClean( frame );

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

