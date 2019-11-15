#include "src/common/precompiled.h"

#include "world.h"

#include "worldpoint.h"

#include "frame.h"

#include "src/common/functions.h"

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
    cv::moveWindow( "Stereo", 900, 10 );

    cv::namedWindow( "Track", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "Track", 800, 600 );
    cv::moveWindow( "Track", 80, 900 );
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

    if ( !m_frames.empty() ) {

        auto previousStereoFrame = std::dynamic_pointer_cast< StereoFrame >( m_frames.back() );

        auto consecutiveLeftFrame = AdjacentFrame::create();
        auto consecutiveRightFrame = AdjacentFrame::create();

        consecutiveLeftFrame->matchFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->leftFrame() ),
                                                std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->leftFrame() ) );

        consecutiveRightFrame->matchFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->rightFrame() ),
                                                std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->rightFrame() ) );

        auto leftTrack = consecutiveLeftFrame->drawTrack( leftImage );
        auto rightTrack = consecutiveRightFrame->drawTrack( rightImage );

        std::cout << stereoFrame->leftFrame()->tracksCount() << " " << stereoFrame->rightFrame()->tracksCount() << std::endl;

        auto track = stackImages( leftTrack, rightTrack );

        cv::imshow( "Track", track );

    }

    m_frames.push_back( stereoFrame );

    cv::waitKey(15);

    return true;

}

}
