#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"

#include "frame.h"

#include "world.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

namespace slam {

// Map
Map::Map( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld )
    :  m_parentWorld( parentWorld ), m_projectionMatrix( projectionMatrix )
{
    initialize();
}

void Map::initialize() {
}

Map::WorldPtr Map::parentWorld() const
{
    return m_parentWorld.lock();
}

const cv::Mat Map::baselineVector() const
{
    return m_projectionMatrix.baselineVector();
}

double Map::baselineLenght() const
{
    return cv::norm( baselineVector() );
}

double Map::minTriangulateCameraDistance() const
{
    return baselineLenght() * parentWorld()->minAdjacentCameraMultiplier();
}

Map::MapPointPtr Map::createMapPoint( const cv::Point3d &pt, const cv::Scalar &color )
{
    auto point = MapPoint::create( shared_from_this(), pt, color );

    addMapPoint( point );

    return point;

}

void Map::removeMapPoint( const MapPointPtr &point )
{
    m_mapPoints.erase( point );
}

void Map::addMapPoint( const MapPointPtr &point )
{
    m_mapPoints.insert( point );
}

std::set< Map::MapPointPtr > Map::mapPoints() const
{
    m_mutex.lock();
    auto ret = m_mapPoints;
    m_mutex.unlock();

    return ret;
}

std::list< Map::FramePtr > Map::frames() const
{
    m_mutex.lock();
    auto ret = m_frames;
    m_mutex.unlock();

    return ret;
}

const Map::FramePtr &Map::backFrame() const
{
    return m_frames.back();
}

void Map::adjust( const int frames )
{
    std::list< FramePtr > list;

    int counter = 0;

    for ( auto i = m_frames.rbegin(); i != m_frames.rend() && counter < frames; ++i, ++counter )
        if ( std::dynamic_pointer_cast< StereoFrame >( *i ) )
            list.push_front( *i );

    m_optimizer.adjust( list );

}

size_t calcTrackLenght( const std::vector< std::shared_ptr< MonoPoint > > &points )
{
    size_t ret = 0;

    for ( auto &i : points )
        if ( i )
            ret = std::max( ret, i->prevTrackLenght() );

    return ret;

}

void Map::adjustLast()
{
    if ( !m_frames.empty() ) {

        std::list< FramePtr > list;

        auto backFrame = m_frames.back();

        if ( backFrame ) {

            auto leftFrame = backFrame->leftFrame();
            auto rightFrame = backFrame->rightFrame();

            if ( leftFrame && rightFrame ) {

                auto count = std::max( calcTrackLenght( leftFrame->framePoints() ), calcTrackLenght( rightFrame->framePoints() ) );

                size_t counter = 0;

                for ( auto i = m_frames.rbegin(); i != m_frames.rend() && counter < count; ++i, ++counter )
                    list.push_front( *i );

                m_optimizer.adjust( list );

            }

        }

    }

}

bool Map::isRudimental() const
{
    return m_frames.size() <= 1;
}

// FlowMap
FlowMap::FlowMap( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld )
    : Map( projectionMatrix, parentWorld )
{
}

FlowMap::ObjectPtr FlowMap::create( const StereoCameraMatrix &cameraMatrix , const WorldPtr &parentWorld )
{
    return ObjectPtr( new FlowMap( cameraMatrix, parentWorld ) );
}

std::shared_ptr< FlowMap > FlowMap::shared_from_this()
{
    return std::dynamic_pointer_cast< FlowMap >( Map::shared_from_this() );
}

std::shared_ptr< const FlowMap > FlowMap::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FlowMap >( Map::shared_from_this() );
}

bool FlowMap::track( const StampedImage &leftImage, const StampedImage &rightImage )
{    
    auto denseFrame = FlowDenseFrame::create( shared_from_this() );

    denseFrame->load( leftImage, rightImage );

    if ( m_denseFlag && m_frames.size() % m_denseStep == 0 )
        denseFrame->processDenseCloud();

    const std::lock_guard< std::mutex > lock( m_mutex );

    if ( m_frames.empty() ) {

        denseFrame->setProjectionMatrix( m_projectionMatrix );
        m_frames.push_back( denseFrame );

    }
    else {

        auto previousDenseFrame = std::dynamic_pointer_cast< FlowDenseFrame >( m_frames.back() );

        if ( previousDenseFrame ) {

            auto goodTrackInliersRatio = parentWorld()->goodTrackInliersRatio();
            auto minTrackInliersRatio = parentWorld()->minTrackInliersRatio();

            auto previousLeftFrame = std::dynamic_pointer_cast< FlowFrame >( previousDenseFrame->leftFrame() );
            auto previousRightFrame = std::dynamic_pointer_cast< FlowFrame >( previousDenseFrame->rightFrame() );
            auto leftFrame = std::dynamic_pointer_cast< FlowFrame >( denseFrame->leftFrame() );
            auto rightFrame = std::dynamic_pointer_cast< FlowFrame >( denseFrame->rightFrame() );

            auto adjacentLeftFrame = FlowConsecutiveFrame::create( shared_from_this() );

            adjacentLeftFrame->setFrames( previousLeftFrame, leftFrame );

            previousLeftFrame->triangulatePoints();

            adjacentLeftFrame->track();

            size_t trackedPointCount = adjacentLeftFrame->trackedPointsCount();

            double inliersRatio = 0.;

            size_t createPointsCount = m_goodTrackPoints;

            if ( trackedPointCount >= m_goodTrackPoints )
                inliersRatio = adjacentLeftFrame->recoverPose();
            else
                createPointsCount = m_goodTrackPoints - trackedPointCount;

            if ( inliersRatio < goodTrackInliersRatio ) {

                do {

                    do {

                        createPointsCount *= 3;

                        adjacentLeftFrame->createFramePoints( createPointsCount );

                        previousDenseFrame->match();

                        adjacentLeftFrame->track();

                        trackedPointCount = adjacentLeftFrame->trackedPointsCount();

                        std::cout << "Use points count: " << previousLeftFrame->usePointsCount() << " of " << previousLeftFrame->extractedPointsCount() << std::endl;

                        std::cout << "Stereo points count: " << previousDenseFrame->stereoPointsCount() << std::endl;
                        std::cout << "Adjacent points count: " << adjacentLeftFrame->adjacentPointsCount() << std::endl;
                        std::cout << "Tracked points count: " << trackedPointCount << std::endl;

                    } while( trackedPointCount < m_goodTrackPoints && previousLeftFrame->usePointsCount() < previousLeftFrame->extractedPointsCount() );

                    previousDenseFrame->triangulatePoints();

                    inliersRatio = adjacentLeftFrame->recoverPose();

                    std::cout << "Inliers: " << inliersRatio << std::endl;

                } while ( inliersRatio < goodTrackInliersRatio && previousLeftFrame->usePointsCount() < previousLeftFrame->extractedPointsCount() );

            }

            if ( trackedPointCount < m_minTrackPoints || inliersRatio < minTrackInliersRatio )
                return false;

            previousDenseFrame->cleanMapPoints();

            denseFrame->removeExtraPoints( m_goodTrackPoints, m_trackFramePointsCount );

            rightFrame->setCameraMatrix( previousRightFrame->cameraMatrix() );
            rightFrame->setRotation( leftFrame->rotation() );
            rightFrame->setTranslation( leftFrame->translation() + baselineVector() );

            auto replacedFrame = DenseFrame::create( shared_from_this() );
            replacedFrame->replaceAndClean( previousDenseFrame );

            m_frames.back() = replacedFrame;

        }

        m_frames.push_back( denseFrame );

        // adjustLast();
        adjust( m_adjustFramesCount );

    }

    return true;

}

// FeatureMap
FeatureMap::FeatureMap( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld )
    :  Map( projectionMatrix, parentWorld )
{
}

FeatureMap::ObjectPtr FeatureMap::create( const StereoCameraMatrix &cameraMatrix , const WorldPtr &parentWorld )
{
    return ObjectPtr( new FeatureMap( cameraMatrix, parentWorld ) );
}

std::shared_ptr< FeatureMap > FeatureMap::shared_from_this()
{
    return std::dynamic_pointer_cast< FeatureMap >( Map::shared_from_this() );
}

std::shared_ptr< const FeatureMap > FeatureMap::shared_from_this() const
{
    return std::dynamic_pointer_cast< const FeatureMap >( Map::shared_from_this() );
}

bool FeatureMap::track( const StampedImage &leftImage, const StampedImage &rightImage )
{
    return false;
}

}

