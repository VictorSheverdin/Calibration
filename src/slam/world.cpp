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

CvImage World::opticalFlowImage() const
{
    return m_opticalFlowImage;
}

void World::addWorldPoint( const WorldPointPtr &point )
{
    m_worldPoints.push_back( point );
}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    auto stereoFrame = WorldStereoFrame::create( shared_from_this() );

    auto timeStart = std::chrono::system_clock::now();

    stereoFrame->load( leftImage, rightImage );
    stereoFrame->matchFrames();

    m_keyPointsImage = stereoFrame->drawKeyPoints( leftImage, rightImage );
    m_stereoPointsImage = stereoFrame->drawStereoPoints( leftImage, rightImage );

    static cv::Mat baselineVector;

    static CvImage prevLeftImage;
    static CvImage prevRightImage;

    if ( !m_frames.empty() ) {

        auto previousStereoFrame = std::dynamic_pointer_cast< StereoFrame >( m_frames.back() );

        auto consecutiveLeftFrame = WorldAdjacentFrame::create( shared_from_this() );
        auto consecutiveRightFrame = WorldAdjacentFrame::create( shared_from_this() );

        consecutiveLeftFrame->setFrames( std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->leftFrame() ),
                                                std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->leftFrame() ) );
        consecutiveLeftFrame->matchFrames();

        consecutiveRightFrame->setFrames( std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->rightFrame() ),
                                                std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->rightFrame() ) );
        consecutiveRightFrame->matchFrames();

        std::cout << "Matching time: " << std::chrono::duration_cast< std::chrono::microseconds >( std::chrono::system_clock::now() - timeStart ).count() << std::endl;

        timeStart = std::chrono::system_clock::now();

        consecutiveLeftFrame->calcOpticalFlow( prevLeftImage, leftImage );
        consecutiveRightFrame->calcOpticalFlow( prevRightImage, rightImage );

        if ( consecutiveLeftFrame->previousFrame()->adjacentPoints().size() > consecutiveRightFrame->previousFrame()->adjacentPoints().size() ) {
            consecutiveLeftFrame->track();
            consecutiveRightFrame->track();
            consecutiveRightFrame->nextFrame()->setCameraMatrix( consecutiveRightFrame->previousFrame()->cameraMatrix() );
            consecutiveRightFrame->nextFrame()->setRotation( consecutiveLeftFrame->nextFrame()->rotation() );
            consecutiveRightFrame->nextFrame()->setTranslation( consecutiveLeftFrame->nextFrame()->translation() + baselineVector );

            //consecutiveRightFrame->triangulatePoints();

        }
        else {
            consecutiveLeftFrame->track();
            consecutiveRightFrame->track();
            consecutiveLeftFrame->nextFrame()->setCameraMatrix( consecutiveLeftFrame->previousFrame()->cameraMatrix() );
            consecutiveLeftFrame->nextFrame()->setRotation( consecutiveRightFrame->nextFrame()->rotation() );
            consecutiveLeftFrame->nextFrame()->setTranslation( consecutiveRightFrame->nextFrame()->translation() - baselineVector );

            //consecutiveLeftFrame->triangulatePoints();
        }

        stereoFrame->triangulatePoints();

        std::cout << "Triangulation time: " << std::chrono::duration_cast< std::chrono::microseconds >( std::chrono::system_clock::now() - timeStart ).count() << std::endl;
        std::cout << std::endl;

        auto leftTrack = consecutiveLeftFrame->drawTrack( leftImage );
        auto rightTrack = consecutiveRightFrame->drawTrack( rightImage );

        m_tracksImage = stackImages( leftTrack, rightTrack );

        auto leftFlow = consecutiveLeftFrame->drawOpticalFrow( leftImage );
        auto rightFlow = consecutiveRightFrame->drawOpticalFrow( rightImage );

        m_opticalFlowImage = stackImages( leftFlow, rightFlow );

        m_frames.push_back( stereoFrame );

        prevLeftImage = leftImage;
        prevRightImage = rightImage;
    }
    else {
        auto leftFrame = stereoFrame->leftFrame();
        auto rightFrame = stereoFrame->rightFrame();

        leftFrame->setProjectionMatrix( m_calibration.leftProjectionMatrix() );
        rightFrame->setProjectionMatrix( m_calibration.rightProjectionMatrix() );

        auto cameraMatrix = leftFrame->cameraMatrix();

        cameraMatrix.at< double >( 0, 0 ) = cameraMatrix.at< double >( 0, 0 ) / 2.0;
        cameraMatrix.at< double >( 1, 1 ) = cameraMatrix.at< double >( 1, 1 ) / 2.0;
        cameraMatrix.at< double >( 0, 2 ) = cameraMatrix.at< double >( 0, 2 ) / 2.0;
        cameraMatrix.at< double >( 1, 2 ) = cameraMatrix.at< double >( 1, 2 ) / 2.0;

        leftFrame->setCameraMatrix( cameraMatrix );
        rightFrame->setCameraMatrix( cameraMatrix );

        baselineVector = rightFrame->translation();

        stereoFrame->triangulatePoints();

        m_frames.push_back( stereoFrame );

        prevLeftImage = leftImage;
        prevRightImage = rightImage;

    }

    return true;

}

}
