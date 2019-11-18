#include "src/common/precompiled.h"

#include "world.h"

#include "worldpoint.h"

#include "frame.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

#include <opencv2/viz.hpp>

namespace slam {

World::World( const StereoCalibrationDataShort &calibration )
    : m_calibration( calibration )
{
    initialize();
}

void World::initialize()
{
    cv::namedWindow( "KeyPoints", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "KeyPoints", 800, 600 );
    cv::moveWindow( "KeyPoints", 80, 10 );

    cv::namedWindow( "Stereo", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "Stereo", 800, 600 );
    cv::moveWindow( "Stereo", 880, 10 );

    cv::namedWindow( "Track", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "Track", 800, 600 );
    cv::moveWindow( "Track", 80, 900 );

    m_vizWindow = std::make_shared< cv::viz::Viz3d >( "Viz3d" );
    m_vizWindow->showWidget( "coordSystemWidget", cv::viz::WCoordinateSystem() );
    m_vizWindow->setWindowPosition( cv::Point( 880, 900 ) );
    m_vizWindow->setWindowSize( cv::Size( 800, 600 ) );

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

std::vector< World::WorldPointPtr > &World::worldPoints()
{
    return m_worldPoints;
}

const std::vector< World::WorldPointPtr > &World::worldPoints() const
{
    return m_worldPoints;
}

void World::addWorldPoint( const WorldPointPtr &point )
{
    m_worldPoints.push_back( point );
}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    auto stereoFrame = StereoFrame::create();

    stereoFrame->load( leftImage, rightImage  );

    auto keyPoints = stereoFrame->drawKeyPoints( leftImage, rightImage );
    cv::imshow( "KeyPoints", keyPoints );

    auto stereoCorrespondencies = stereoFrame->drawStereoPoints( leftImage, rightImage );
    cv::imshow( "Stereo", stereoCorrespondencies );

    stereoFrame->leftFrame()->setCameraMatrix( m_calibration.leftCameraResults().cameraMatrix() );

    stereoFrame->rightFrame()->setCameraMatrix( m_calibration.rightCameraResults().cameraMatrix() );
    stereoFrame->rightFrame()->setTranslation( m_calibration.translationVector() );
    stereoFrame->rightFrame()->setRotation( m_calibration.rotationMatrix() );

    auto stereoPoints = stereoFrame->triangulatePoints( leftImage );

    std::vector< cv::Vec3d > points;
    std::vector< cv::Vec4b > colors;

    for ( auto &i : stereoPoints ) {
        points.push_back( i.spatialPoint() );
        colors.push_back( i.spatialColor() );
    }

    if ( !points.empty() ) {
        cv::viz::WCloud cloud( points, colors );
        cloud.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
        m_vizWindow->showWidget( "Point cloud", cloud );

    }

    std::vector< cv::Affine3d > trajectoryPoints;

    cv::Affine3d cameraPose;
    trajectoryPoints.push_back( cameraPose );

    cv::viz::WTrajectory trajectory( trajectoryPoints );
    m_vizWindow->showWidget( "trajectory", trajectory );

    cv::viz::WTrajectoryFrustums trajectoryFrustums( trajectoryPoints, cv::Vec2d( 1, 1 ), 1 );
    m_vizWindow->showWidget( "trajectoryFrustums", trajectoryFrustums );

    if ( !m_frames.empty() ) {

        auto previousStereoFrame = std::dynamic_pointer_cast< StereoFrame >( m_frames.back() );

        auto consecutiveLeftFrame = AdjacentFrame::create();
        auto consecutiveRightFrame = AdjacentFrame::create();

        consecutiveLeftFrame->matchFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->leftFrame() ), std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->leftFrame() ) );
        consecutiveRightFrame->matchFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->rightFrame() ), std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->rightFrame() ) );

        auto leftTrack = consecutiveLeftFrame->drawTrack( leftImage );
        auto rightTrack = consecutiveRightFrame->drawTrack( rightImage );

        std::cout << stereoFrame->leftFrame()->tracksCount() << " " << stereoFrame->rightFrame()->tracksCount() << std::endl;

        auto track = stackImages( leftTrack, rightTrack );

        cv::imshow( "Track", track );

    }

    m_frames.push_back( stereoFrame );

    m_vizWindow->spinOnce();
    cv::waitKey( 1 );

    return true;

}

}
