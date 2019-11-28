#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"

#include "frame.h"

#include "world.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

namespace slam {

Map::Map(const WorldPtr &world )
    : m_parentWorld( world )
{
    initialize();
}

void Map::initialize()
{
}

Map::MapPtr Map::create( const WorldPtr &world )
{
    return MapPtr( new Map( world ) );
}

Map::MapPointPtr Map::createMapPoint()
{
    auto point = MapPoint::create( shared_from_this() );

    addMapPoint( point );

    return point;

}

Map::MapPointPtr Map::createMapPoint( const cv::Vec3f &pt )
{
    auto point = MapPoint::create( shared_from_this(), pt );

    addMapPoint( point );

    return point;

}

Map::MapPointPtr Map::createMapPoint( const cv::Vec3f &pt, const cv::Scalar &color )
{
    auto point = MapPoint::create( shared_from_this(), pt, color );

    addMapPoint( point );

    return point;

}

std::list< Map::FramePtr > &Map::frames()
{
    return m_frames;
}

const std::list< Map::FramePtr > &Map::frames() const
{
    return m_frames;
}

std::list< Map::MapPointPtr > &Map::mapPoints()
{
    return m_mapPoints;
}

const std::list< Map::MapPointPtr > &Map::mapPoints() const
{
    return m_mapPoints;
}

const Map::FramePtr &Map::backFrame() const
{
    return m_frames.back();
}

CvImage Map::keyPointsImage() const
{
    return m_keyPointsImage;
}

CvImage Map::stereoPointsImage() const
{
    return m_stereoPointsImage;
}

CvImage Map::tracksImage() const
{
    return m_tracksImage;
}

void Map::addMapPoint( const MapPointPtr &point )
{
    m_mapPoints.push_back( point );
}

bool Map::track( const CvImage &leftImage, const CvImage &rightImage )
{
    if ( m_parentWorld.expired() )
        return false;

    auto parentWorld = m_parentWorld.lock();

    auto scaleFactor = parentWorld->scaleFactor();

    CvImage resizedLeftImage;
    CvImage resizedRightImage;

    if ( std::abs( scaleFactor - 1.0 ) > DOUBLE_EPS ) {
        cv::resize( leftImage, resizedLeftImage, cv::Size(), scaleFactor, scaleFactor );
        cv::resize( rightImage, resizedRightImage, cv::Size(), scaleFactor, scaleFactor );
    }
    else {
        resizedLeftImage = leftImage;
        resizedRightImage = rightImage;
    }

    auto stereoFrame = MapStereoFrame::create( shared_from_this() );

    auto timeStart = std::chrono::system_clock::now();

    stereoFrame->load( resizedLeftImage, resizedRightImage );
    stereoFrame->extractKeyPoints();
    stereoFrame->matchOptical();

    m_keyPointsImage = stereoFrame->drawKeyPoints();
    m_stereoPointsImage = stereoFrame->drawStereoPoints();

    static cv::Mat baselineVector;

    if ( m_frames.empty() ) {

        auto leftFrame = stereoFrame->leftFrame();
        auto rightFrame = stereoFrame->rightFrame();

        leftFrame->setProjectionMatrix( parentWorld->calibration().leftProjectionMatrix() );
        rightFrame->setProjectionMatrix( parentWorld->calibration().rightProjectionMatrix() );

        auto cropRect = parentWorld->calibration().cropRect();
        auto principal = cv::Vec2f( -cropRect.x, -cropRect.y );

        leftFrame->movePrincipalPoint( principal );
        rightFrame->movePrincipalPoint( principal );

        if ( std::abs( scaleFactor - 1.0 ) > DOUBLE_EPS ) {
            leftFrame->multiplicateCameraMatrix( scaleFactor );
            rightFrame->multiplicateCameraMatrix( scaleFactor );
        }

        baselineVector = rightFrame->translation();

        stereoFrame->triangulatePoints();

        m_frames.push_back( stereoFrame );

    }
    else {

        auto previousStereoFrame = std::dynamic_pointer_cast< StereoFrame >( m_frames.back() );

        auto consecutiveLeftFrame = MapAdjacentFrame::create( shared_from_this() );
        auto consecutiveRightFrame = MapAdjacentFrame::create( shared_from_this() );

        consecutiveLeftFrame->setFrames( std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->leftFrame() ),
                                                std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->leftFrame() ) );

        consecutiveLeftFrame->matchOptical();

        consecutiveRightFrame->setFrames( std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->rightFrame() ),
                                                std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->rightFrame() ) );

        consecutiveRightFrame->matchOptical();

        auto matchingTime = std::chrono::system_clock::now();

        if ( consecutiveLeftFrame->previousFrame()->adjacentPoints().size() > consecutiveRightFrame->previousFrame()->adjacentPoints().size() ) {
            consecutiveLeftFrame->track();
            consecutiveRightFrame->nextFrame()->setCameraMatrix( consecutiveRightFrame->previousFrame()->cameraMatrix() );
            consecutiveRightFrame->nextFrame()->setRotation( consecutiveLeftFrame->nextFrame()->rotation() );
            consecutiveRightFrame->nextFrame()->setTranslation( consecutiveLeftFrame->nextFrame()->translation() + baselineVector );

            // consecutiveRightFrame->triangulatePoints();

        }
        else {
            consecutiveRightFrame->track();
            consecutiveLeftFrame->nextFrame()->setCameraMatrix( consecutiveLeftFrame->previousFrame()->cameraMatrix() );
            consecutiveLeftFrame->nextFrame()->setRotation( consecutiveRightFrame->nextFrame()->rotation() );
            consecutiveLeftFrame->nextFrame()->setTranslation( consecutiveRightFrame->nextFrame()->translation() - baselineVector );

            // consecutiveLeftFrame->triangulatePoints();

        }

        stereoFrame->triangulatePoints();

        auto calcTime = std::chrono::system_clock::now();
        std::cout << "Matching time: "
                  << std::chrono::duration_cast< std::chrono::microseconds >( matchingTime - timeStart ).count() / 1.e6
                  << " sec. ";
        std::cout << "Calculation time: "
                  << std::chrono::duration_cast< std::chrono::microseconds >( calcTime - matchingTime ).count() / 1.e6
                  << " sec." << std::endl;

        std::cout << std::endl;

        auto leftTrack = consecutiveLeftFrame->drawTrack();
        auto rightTrack = consecutiveRightFrame->drawTrack();

        m_tracksImage = stackImages( leftTrack, rightTrack );

        previousStereoFrame->clearImages();

        m_frames.push_back( stereoFrame );

    }

    return true;

}

}
