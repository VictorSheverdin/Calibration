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
    m_previousKeypointsCount = m_goodTrackPoints * 2;
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

void Map::addMapPoint( const MapPointPtr &point )
{
    m_mapPoints.insert( point );
}

Map::WorldPtr Map::parentWorld() const
{
    return m_parentWorld.lock();
}

const cv::Mat &Map::baselineVector() const
{
    return parentWorld()->baselineVector();
}

double Map::baselineLenght() const
{
    return cv::norm( baselineVector() );
}

bool Map::track( const CvImage &leftImage, const CvImage &rightImage )
{
    auto parentWorld = this->parentWorld();

    if ( !parentWorld )
        return false;

    auto stereoFrame = ProcessedStereoFrame::create( shared_from_this() );

    stereoFrame->load( leftImage, rightImage );

    if ( m_frames.empty() ) {

        stereoFrame->extractKeyPoints();

        stereoFrame->setProjectionMatrix( parentWorld->leftProjectionMatrix(), parentWorld->rightProjectionMatrix() );

        m_frames.push_back( stereoFrame );

    }
    else {

        auto consecutiveLeftFrame = AdjacentFrame::create();

        auto previousStereoFrame = std::dynamic_pointer_cast< ProcessedStereoFrame >( m_frames.back() );

        if ( previousStereoFrame ) {

            auto previousLeftFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->leftFrame() );
            auto previousRightFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->rightFrame() );
            auto nextLeftFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->leftFrame() );
            auto nextRightFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->rightFrame() );

            consecutiveLeftFrame->setFrames( previousLeftFrame, nextLeftFrame );

            stereoFrame->extractKeyPoints();

            int keypointsCount = m_previousKeypointsCount;

            int matchedPointCount;
            bool continueFlag = true;

            do {

                continueFlag = continueFlag && previousStereoFrame->matchOptical( keypointsCount );
                continueFlag = continueFlag && consecutiveLeftFrame->matchOptical( keypointsCount );

                m_previousKeypointsCount = keypointsCount;

                keypointsCount *= 2;

                matchedPointCount = consecutiveLeftFrame->matchedPointsCount();

            } while( matchedPointCount < m_goodTrackPoints && continueFlag );

            std::cout << matchedPointCount << std::endl;

            if ( matchedPointCount > m_minTrackPoints ) {

                if ( matchedPointCount > m_overTrackPoints )
                    m_previousKeypointsCount /= 2;

                previousStereoFrame->triangulatePoints();

                consecutiveLeftFrame->track();
                nextRightFrame->setCameraMatrix( previousRightFrame->cameraMatrix() );
                nextRightFrame->setRotation( nextLeftFrame->rotation() );
                nextRightFrame->setTranslation( nextLeftFrame->translation() + baselineVector() );

                nextLeftFrame->triangulatePoints();

                previousStereoFrame->clearMapPoints();

                auto replacedFrame = StereoFrame::create();

                replacedFrame->replace( previousStereoFrame );

                m_frames.pop_back();

                m_frames.push_back( replacedFrame );

                m_frames.push_back( stereoFrame );

            }

        }

    }

    return true;

}

}

