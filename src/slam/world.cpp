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

std::vector< World::WorldPointPtr > &World::worldPoints()
{
    return m_worldPoints;
}

const std::vector< World::WorldPointPtr > &World::worldPoints() const
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

    stereoFrame->load( leftImage, rightImage  );
    stereoFrame->matchFrames();

    m_keyPointsImage = stereoFrame->drawKeyPoints( leftImage, rightImage );
    m_stereoPointsImage = stereoFrame->drawStereoPoints( leftImage, rightImage );

    if ( !m_frames.empty() ) {

        auto previousStereoFrame = std::dynamic_pointer_cast< StereoFrame >( m_frames.back() );

        auto consecutiveLeftFrame = WorldAdjacentFrame::create( shared_from_this() );
        auto consecutiveRightFrame = WorldAdjacentFrame::create( shared_from_this() );

        consecutiveLeftFrame->matchFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->leftFrame() ), std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->leftFrame() ) );
        consecutiveRightFrame->matchFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->rightFrame() ), std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->rightFrame() ) );

        std::vector< cv::Point3f > worldPoints;
        std::vector< cv::Point2f > framePoints;

        auto trackFrame = consecutiveLeftFrame->frame2();

        for ( auto &i : trackFrame->framePoints() ) {
            if ( i && i->worldPoint() ) {
                worldPoints.push_back( i->worldPoint()->point() );
                framePoints.push_back( i->point() );

            }

        }

        if ( framePoints.size() >= m_minPnpPoints ) {

            cv::Mat rvec;
            cv::Mat tvec;

            cv::solvePnPRansac( worldPoints, framePoints, m_calibration.leftCameraResults().cameraMatrix(), cv::noArray(), rvec, tvec );

            cv::Mat rmat;
            cv::Rodrigues( rvec, rmat );

            auto leftFrame = stereoFrame->leftFrame();
            auto rightFrame = stereoFrame->rightFrame();

            leftFrame->setCameraMatrix( m_calibration.leftCameraResults().cameraMatrix() );
            leftFrame->setTranslation( tvec );
            leftFrame->setRotation( rmat );

            rightFrame->setCameraMatrix( m_calibration.rightCameraResults().cameraMatrix() );
            rightFrame->setTranslation( tvec + m_calibration.translationVector() );
            rightFrame->setRotation( m_calibration.rotationMatrix() * rmat );

            /*std::cout << std::endl;
            std::cout << framePoints.size() << std::endl;
            std::cout << "T:" << tvec << std::endl;
            std::cout << "R:" << rmat << std::endl;
            std::cout << std::endl;*/

            stereoFrame->triangulatePoints();

            m_frames.push_back( stereoFrame );

        }

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
        std::cout << leftFrame->projectionMatrix() << std::endl;
        leftFrame->setProjectionMatrix( m_calibration.leftProjectionMatrix() );
        std::cout << leftFrame->projectionMatrix() << std::endl;
        std::cout <<  std::endl;

        rightFrame->setCameraMatrix( m_calibration.rightCameraResults().cameraMatrix() );
        rightFrame->setTranslation( m_calibration.baselineVector() );
        rightFrame->setRotation( cv::Mat::eye( 3, 3, CV_64F ) );
        std::cout << rightFrame->projectionMatrix() << std::endl;
        rightFrame->setProjectionMatrix( m_calibration.rightProjectionMatrix() );
        std::cout << rightFrame->projectionMatrix() << std::endl;

        stereoFrame->triangulatePoints();

        m_frames.push_back( stereoFrame );

    }

    return true;

}

}
