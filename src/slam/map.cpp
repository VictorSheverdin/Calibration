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

void Map::removeMapPoint( const MapPointPtr &point )
{
    m_mapPoints.erase( point );
}

std::list< Map::FramePtr > &Map::frames()
{
    return m_frames;
}

const std::list< Map::FramePtr > &Map::frames() const
{
    return m_frames;
}

std::set< Map::MapPointPtr > &Map::mapPoints()
{
    return m_mapPoints;
}

const std::set< Map::MapPointPtr > &Map::mapPoints() const
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
    m_mapPoints.insert( point );
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
        cv::resize( leftImage, resizedLeftImage, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC );
        cv::resize( rightImage, resizedRightImage, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC );
    }
    else {
        resizedLeftImage = leftImage;
        resizedRightImage = rightImage;
    }

    auto stereoFrame = ProcessedStereoFrame::create( shared_from_this() );

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

        auto previousStereoFrame = std::dynamic_pointer_cast< ProcessedStereoFrame >( m_frames.back() );

        auto consecutiveLeftFrame = AdjacentFrame::create();
        auto consecutiveRightFrame = AdjacentFrame::create();

        auto previousLeftFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->leftFrame() );
        auto previousRightFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->rightFrame() );
        auto nextLeftFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->leftFrame() );
        auto nextRightFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->rightFrame() );

        consecutiveLeftFrame->setFrames( previousLeftFrame, nextLeftFrame );

        consecutiveLeftFrame->matchOptical();

        consecutiveRightFrame->setFrames( previousRightFrame, nextRightFrame );

        consecutiveRightFrame->matchOptical();

        auto matchingTime = std::chrono::system_clock::now();

        if ( consecutiveLeftFrame->previousFrame()->adjacentPoints().size() > consecutiveRightFrame->previousFrame()->adjacentPoints().size() ) {
            consecutiveLeftFrame->track();
            consecutiveRightFrame->nextFrame()->setCameraMatrix( consecutiveRightFrame->previousFrame()->cameraMatrix() );
            consecutiveRightFrame->nextFrame()->setRotation( consecutiveLeftFrame->nextFrame()->rotation() );
            consecutiveRightFrame->nextFrame()->setTranslation( consecutiveLeftFrame->nextFrame()->translation() + baselineVector );

        }
        else {
            consecutiveRightFrame->track();
            consecutiveLeftFrame->nextFrame()->setCameraMatrix( consecutiveLeftFrame->previousFrame()->cameraMatrix() );
            consecutiveLeftFrame->nextFrame()->setRotation( consecutiveRightFrame->nextFrame()->rotation() );
            consecutiveLeftFrame->nextFrame()->setTranslation( consecutiveRightFrame->nextFrame()->translation() - baselineVector );

        }

        nextLeftFrame->triangulatePoints();
        nextRightFrame->triangulatePoints();

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
        previousStereoFrame->clearMapPoints();

        m_frames.push_back( stereoFrame );

    }

    return true;

}

}
