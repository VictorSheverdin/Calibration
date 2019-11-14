#include "src/common/precompiled.h"

#include "world.h"

#include "worldpoint.h"

#include "frame.h"

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

    cv::namedWindow( "StereoPoint", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "StereoPoint", 800, 600 );
    cv::moveWindow( "StereoPoint", 900, 10 );

    cv::namedWindow( "ConsequitivePoints", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "ConsequitivePoints", 800, 600 );
    cv::moveWindow( "ConsequitivePoints", 80, 900 );
}

World::WorldPtr World::create( const StereoCalibrationDataShort &calibration )
{
    return WorldPtr( new World( calibration ) );
}

World::WorldPointPtr World::createWorldPoint()
{
    return WorldPoint::create( shared_from_this() );
}

World::WorldPointPtr World::createWorldPoint( const cv::Vec3f &pt )
{
    return WorldPoint::create( shared_from_this(), pt );
}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    auto stereoFrame = StereoFrame::create();

    stereoFrame->load( leftImage, rightImage  );

    auto keyPoints = stereoFrame->drawKeyPoints( leftImage, rightImage );
    cv::imshow( "KeyPoints", keyPoints );

    auto stereoCorrespondencies = stereoFrame->drawStereoPoints( leftImage, rightImage );
    cv::imshow( "StereoPoint", stereoCorrespondencies );

    if ( !m_frames.empty() ) {

        auto previousStereoFrame = std::dynamic_pointer_cast< StereoFrame >( m_frames.back() );

        auto consecutiveFrame = ConsecutiveFrame::create();

        consecutiveFrame->matchFrames( std::dynamic_pointer_cast< FeatureFrame >( previousStereoFrame->leftFrame() ),
                                                std::dynamic_pointer_cast< FeatureFrame >( stereoFrame->leftFrame() ) );

        auto track = consecutiveFrame->drawTrack( leftImage );
        cv::imshow( "ConsequitivePoints", track );

    }

    m_frames.push_back( stereoFrame );

    cv::waitKey(15);

    return true;

}

}
