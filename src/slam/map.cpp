#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"

#include "frame.h"

#include "world.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

namespace slam {

const double Map::m_minTrackInliersRatio = 0.9;

Map::Map( const StereoCameraMatrix &projectionMatrix )
    :  m_projectionMatrix( projectionMatrix )
{
    initialize();
}

void Map::initialize()
{
    m_previousKeypointsCount = m_goodTrackPoints * 2;
}

Map::MapPtr Map::create( const StereoCameraMatrix &cameraMatrix )
{
    return MapPtr( new Map( cameraMatrix ) );
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

const cv::Mat Map::baselineVector() const
{
    return m_projectionMatrix.baselineVector();
}

double Map::baselineLenght() const
{
    return cv::norm( baselineVector() );
}

void Map::adjust( const int frames )
{
    const std::lock_guard< std::mutex > lock( m_mutex );

    m_optimizer.adjustStored( this, frames );
}

void Map::localAdjustment()
{
    const std::lock_guard< std::mutex > lock( m_mutex );

    m_optimizer.localAdjustment( this );
}

bool Map::valid() const
{
    return true;
}

bool Map::track( const CvImage &leftImage, const CvImage &rightImage )
{
    ProcessedStereoFrame::FramePtr stereoFrame;
    ProcessedDenseFrame::FramePtr denseFrame;

    if ( m_frames.size() % 1 == 0 ) {
        denseFrame = ProcessedDenseFrame::create( shared_from_this() );
        stereoFrame = denseFrame;
    }
    else
        stereoFrame = ProcessedStereoFrame::create( shared_from_this() );

    stereoFrame->load( leftImage, rightImage );

    stereoFrame->extractKeyPoints();
    stereoFrame->extractGradientPoints();

    if ( denseFrame )
        denseFrame->processDenseCloud();

    const std::lock_guard< std::mutex > lock( m_mutex );

    if ( m_frames.empty() ) {

        stereoFrame->setProjectionMatrix( m_projectionMatrix );
        m_frames.push_back( stereoFrame );

    }
    else {

        auto previousStereoFrame = std::dynamic_pointer_cast< ProcessedStereoFrame >( m_frames.back() );
        auto previousDenseFrame = std::dynamic_pointer_cast< ProcessedDenseFrame >( m_frames.back() );

        if ( previousStereoFrame ) {

            auto previousLeftFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->leftFrame() );
            auto previousRightFrame = std::dynamic_pointer_cast< ProcessedFrame >( previousStereoFrame->rightFrame() );
            auto leftFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->leftFrame() );
            auto rightFrame = std::dynamic_pointer_cast< ProcessedFrame >( stereoFrame->rightFrame() );

            auto adjacentLeftFrame = AdjacentFrame::create();

            adjacentLeftFrame->setFrames( previousLeftFrame, leftFrame );

            auto keypointsCount = m_previousKeypointsCount;

            // previousLeftFrame->triangulatePoints();

            int trackedPointCount;

            double inliersRatio = 0.0;

            auto maxKeypointsCount = previousLeftFrame->keyPointsCount();

            do {

                do {

                    previousStereoFrame->matchOptical( keypointsCount );
                    previousStereoFrame->triangulatePoints();

                    auto trackFramePointsCount = std::max( 0, m_trackFramePointsCount - adjacentLeftFrame->trackFramePointsCount() );

                    adjacentLeftFrame->createFramePoints( trackFramePointsCount );

                    adjacentLeftFrame->trackOptical();

                    trackedPointCount = adjacentLeftFrame->posePointsCount();

                    m_previousKeypointsCount = keypointsCount;

                    keypointsCount *= 2;

                } while( trackedPointCount < m_goodTrackPoints && m_previousKeypointsCount <= maxKeypointsCount );

                inliersRatio = adjacentLeftFrame->recoverPose();

            } while ( inliersRatio < m_minTrackInliersRatio && m_previousKeypointsCount <= maxKeypointsCount );

            std::cout << "Inliers: " << inliersRatio << std::endl;

            if ( trackedPointCount < m_minTrackPoints )
                return false;

            if ( trackedPointCount > m_overTrackPoints )
                m_previousKeypointsCount /= 2;

            rightFrame->setCameraMatrix( previousRightFrame->cameraMatrix() );
            rightFrame->setRotation( leftFrame->rotation() );
            rightFrame->setTranslation( leftFrame->translation() + baselineVector() );

            previousStereoFrame->cleanMapPoints();

            if ( previousDenseFrame ) {
                auto replacedFrame = DenseFrame::create( shared_from_this() );
                replacedFrame->replaceAndClean( previousDenseFrame );

                m_frames.back() = replacedFrame;

            }
            else if ( previousStereoFrame ){
                auto replacedFrame = StereoFrame::create( shared_from_this() );
                replacedFrame->replaceAndClean( previousStereoFrame );

                m_frames.back() = replacedFrame;
            }

        }

        m_frames.push_back( stereoFrame );

    }

    return true;

}

}

