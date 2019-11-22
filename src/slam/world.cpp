#include "src/common/precompiled.h"

#include "world.h"

#include "worldpoint.h"

#include "frame.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

namespace slam {

World::World( const StereoCalibrationDataShort &calibration )
    : m_calibration( calibration )
{
    initialize();
}

void World::initialize()
{
}

World::WorldPtr World::create( const StereoCalibrationDataShort &calibration )
{
    return WorldPtr( new World( calibration ) );
}

World::WorldPointPtr World::createWorldPoint()
{
    auto point = WorldPoint::create( shared_from_this() );

    addWorldPoint( point );

    return point;

}

World::WorldPointPtr World::createWorldPoint( const cv::Vec3f &pt )
{
    auto point = WorldPoint::create( shared_from_this(), pt );

    addWorldPoint( point );

    return point;

}

World::WorldPointPtr World::createWorldPoint( const cv::Vec3f &pt, const cv::Scalar &color )
{
    auto point = WorldPoint::create( shared_from_this(), pt, color );

    addWorldPoint( point );

    return point;

}

std::list< World::FramePtr > &World::frames()
{
    return m_frames;
}

const std::list< World::FramePtr > &World::frames() const
{
    return m_frames;
}

std::list< World::WorldPointPtr > &World::worldPoints()
{
    return m_worldPoints;
}

const std::list< World::WorldPointPtr > &World::worldPoints() const
{
    return m_worldPoints;
}

const World::FramePtr &World::backFrame() const
{
    return m_frames.back();
}

CvImage World::keyPointsImage() const
{
    return m_keyPointsImage;
}

CvImage World::stereoPointsImage() const
{
    return m_stereoPointsImage;
}

CvImage World::tracksImage() const
{
    return m_tracksImage;
}

void World::addWorldPoint( const WorldPointPtr &point )
{
    m_worldPoints.push_back( point );
}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    auto stereoFrame = WorldStereoFrame::create( shared_from_this() );

    stereoFrame->load( leftImage, rightImage );
    stereoFrame->matchFrames();

    m_keyPointsImage = stereoFrame->drawKeyPoints( leftImage, rightImage );
    m_stereoPointsImage = stereoFrame->drawStereoPoints( leftImage, rightImage );

    static cv::Mat baselineVector;

    if ( !m_frames.empty() ) {
        auto previousStereoFrame = std::dynamic_pointer_cast< StereoFrame >( m_frames.back() );

        auto consecutiveLeftFrame = WorldAdjacentFrame::create( shared_from_this() );
        auto consecutiveRightFrame = WorldAdjacentFrame::create( shared_from_this() );

        consecutiveLeftFrame->setFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->leftFrame() ),
                                                std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->leftFrame() ) );
        consecutiveLeftFrame->matchFrames();

        consecutiveRightFrame->setFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->rightFrame() ),
                                                std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->rightFrame() ) );
        consecutiveRightFrame->matchFrames();

        if ( consecutiveLeftFrame->previousFrame()->adjacentPoints().size() > consecutiveRightFrame->previousFrame()->adjacentPoints().size() ) {
            consecutiveLeftFrame->track();
            consecutiveRightFrame->nextFrame()->setCameraMatrix( consecutiveRightFrame->previousFrame()->cameraMatrix() );
            consecutiveRightFrame->nextFrame()->setRotation( consecutiveLeftFrame->nextFrame()->rotation() );
            consecutiveRightFrame->nextFrame()->setTranslation( consecutiveLeftFrame->nextFrame()->translation() + baselineVector );

            // consecutiveLeftFrame->triangulatePoints();

        }
        else {
            consecutiveRightFrame->track();
            consecutiveLeftFrame->nextFrame()->setCameraMatrix( consecutiveLeftFrame->previousFrame()->cameraMatrix() );
            consecutiveLeftFrame->nextFrame()->setRotation( consecutiveRightFrame->nextFrame()->rotation() );
            consecutiveLeftFrame->nextFrame()->setTranslation( consecutiveRightFrame->nextFrame()->translation() - baselineVector );

            // consecutiveRightFrame->triangulatePoints();

        }

        stereoFrame->triangulatePoints();

        m_frames.push_back( stereoFrame );

        auto leftTrack = consecutiveLeftFrame->drawTrack( leftImage );
        auto rightTrack = consecutiveRightFrame->drawTrack( rightImage );

        m_tracksImage = stackImages( leftTrack, rightTrack );

    }
    else {
        auto leftFrame = stereoFrame->leftFrame();
        auto rightFrame = stereoFrame->rightFrame();

        leftFrame->setCameraMatrix( m_calibration.leftCameraResults().cameraMatrix() );
        leftFrame->setTranslation( cv::Mat::zeros( 3, 1, CV_64F ) );
        leftFrame->setRotation( cv::Mat::eye( 3, 3, CV_64F ) );

        rightFrame->setCameraMatrix( m_calibration.rightCameraResults().cameraMatrix() );
        rightFrame->setTranslation( m_calibration.baselineVector() );
        rightFrame->setRotation( cv::Mat::eye( 3, 3, CV_64F ) );

        leftFrame->setProjectionMatrix( m_calibration.leftProjectionMatrix() );
        rightFrame->setProjectionMatrix( m_calibration.rightProjectionMatrix() );

        baselineVector = rightFrame->translation();

        // stereoFrame->matchFrames();

        stereoFrame->triangulatePoints();

        m_frames.push_back( stereoFrame );

    }

    return true;

}

}
