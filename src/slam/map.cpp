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
    ProcessedFrame::setMaxFeatures( m_minKeyPoints );
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

const cv::Mat &Map::baselineVector() const
{
    return m_baselineVector;
}

double Map::baselineLenght() const
{
    return cv::norm( m_baselineVector );
}

bool Map::track( const CvImage &leftImage, const CvImage &rightImage )
{
    if ( m_parentWorld.expired() )
        return false;

    auto parentWorld = m_parentWorld.lock();

    auto scaleFactor = parentWorld->scaleFactor();

    auto stereoFrame = ProcessedStereoFrame::create( shared_from_this() );

    stereoFrame->load( leftImage, rightImage );

    if ( m_frames.empty() ) {

        int stereoPointsCount;
        int keyPointsCount = ProcessedStereoFrame::maxFeatures();

        do {
            stereoFrame->extractKeyPoints();
            stereoFrame->matchOptical();

            stereoPointsCount = stereoFrame->stereoPointsCount();

            if ( stereoPointsCount < m_goodStereoPoints ) {
                keyPointsCount = std::max( m_minKeyPoints, /*m_goodStereoPoints * ProcessedStereoFrame::maxFeatures() / stereoPointsCount*/keyPointsCount * 2 );
                ProcessedStereoFrame::setMaxFeatures( keyPointsCount );
            }

        } while( stereoPointsCount < m_goodStereoPoints && keyPointsCount < m_maxKeyPoints );

        m_keyPointsImage = stereoFrame->drawKeyPoints();
        m_stereoPointsImage = stereoFrame->drawStereoPoints();

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

        m_baselineVector = rightFrame->translation();

        stereoFrame->triangulatePoints();

        m_frames.push_back( stereoFrame );

    }
    else {

        int stereoPointsCount;
        int trackPointsCount;
        int keyPointsCount = ProcessedStereoFrame::maxFeatures();

        auto consecutiveLeftFrame = AdjacentFrame::create();
        auto consecutiveRightFrame = AdjacentFrame::create();

        auto previousStereoFrame = std::dynamic_pointer_cast< ProcessedStereoFrame >( m_frames.back() );

        auto previousLeftFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->leftFrame() );
        auto previousRightFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->rightFrame() );
        auto nextLeftFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->leftFrame() );
        auto nextRightFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->rightFrame() );

        consecutiveLeftFrame->setFrames( previousLeftFrame, nextLeftFrame );
        consecutiveRightFrame->setFrames( previousRightFrame, nextRightFrame );

        do {

            stereoFrame->extractKeyPoints();
            stereoFrame->matchOptical();

            stereoPointsCount = stereoFrame->stereoPointsCount();

            consecutiveLeftFrame->matchOptical();
            consecutiveRightFrame->matchOptical();

            if ( consecutiveLeftFrame->previousFrame()->adjacentPoints().size() > consecutiveRightFrame->previousFrame()->adjacentPoints().size() ) {
                consecutiveLeftFrame->track();
                consecutiveRightFrame->nextFrame()->setCameraMatrix( consecutiveRightFrame->previousFrame()->cameraMatrix() );
                consecutiveRightFrame->nextFrame()->setRotation( consecutiveLeftFrame->nextFrame()->rotation() );
                consecutiveRightFrame->nextFrame()->setTranslation( consecutiveLeftFrame->nextFrame()->translation() + m_baselineVector );

                trackPointsCount = consecutiveLeftFrame->trackPointsCount();

            }
            else {
                consecutiveRightFrame->track();
                consecutiveLeftFrame->nextFrame()->setCameraMatrix( consecutiveLeftFrame->previousFrame()->cameraMatrix() );
                consecutiveLeftFrame->nextFrame()->setRotation( consecutiveRightFrame->nextFrame()->rotation() );
                consecutiveLeftFrame->nextFrame()->setTranslation( consecutiveRightFrame->nextFrame()->translation() - m_baselineVector );

                trackPointsCount = consecutiveLeftFrame->trackPointsCount();

            }

            if ( trackPointsCount < m_goodTrackPoints || stereoPointsCount < m_goodStereoPoints ) {
                keyPointsCount = std::max( m_minKeyPoints, keyPointsCount * 2 );
                ProcessedStereoFrame::setMaxFeatures( keyPointsCount );

            }
            else if ( trackPointsCount > m_overageTrackPoints || stereoPointsCount > m_overageStereoPoints ) {
                keyPointsCount = std::max( m_minKeyPoints, keyPointsCount / 2 );
                ProcessedStereoFrame::setMaxFeatures( keyPointsCount );

            }

        } while( trackPointsCount < m_goodTrackPoints && keyPointsCount < m_maxKeyPoints );

        m_keyPointsImage = stereoFrame->drawKeyPoints();
        m_stereoPointsImage = stereoFrame->drawStereoPoints();

        auto leftTrack = consecutiveLeftFrame->drawTrack();
        auto rightTrack = consecutiveRightFrame->drawTrack();

        m_tracksImage = stackImages( leftTrack, rightTrack );

        // nextLeftFrame->triangulatePoints();
        // nextRightFrame->triangulatePoints();

        stereoFrame->triangulatePoints();

        previousStereoFrame->clearMapPoints();

        auto replacedFrame = StereoFrame::create();

        replacedFrame->replace( previousStereoFrame );

        m_frames.pop_back();

        m_frames.push_back( replacedFrame );

        m_frames.push_back( stereoFrame );

    }

    return true;

}

}
