#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"

#include "frame.h"

#include "world.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

#include <opencv2/sfm.hpp>

namespace slam {

// Map
Map::Map( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld )
    :  m_parentWorld( parentWorld ), m_projectionMatrix( projectionMatrix )
{
    initialize();
}

void Map::initialize()
{
}

Map::ObjectPtr Map::create( const StereoCameraMatrix &cameraMatrix , const WorldPtr &parentWorld )
{
    return ObjectPtr( new Map( cameraMatrix, parentWorld ) );
}

WorldPtr Map::parentWorld() const
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

MapPointPtr Map::createMapPoint( const cv::Point3d &pt, const cv::Scalar &color )
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

const std::set< MapPointPtr > &Map::mapPoints() const
{
    return m_mapPoints;
}

const std::list< StereoFramePtr > &Map::frames() const
{
    return m_frames;
}

const StereoFramePtr &Map::backFrame() const
{
    return m_frames.back();
}

void Map::adjust( const int frames )
{
    std::list< StereoKeyFramePtr > list;

    int counter = 0;

    for ( auto i = m_frames.rbegin(); i != m_frames.rend() && counter < frames; ++i, ++counter ) {
        auto frame = std::dynamic_pointer_cast< StereoKeyFrame >( *i );
        if ( frame )
            list.push_front( frame );
    }

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

        std::list< StereoKeyFramePtr > list;

        auto backFrame = m_frames.back();

        if ( backFrame ) {

            auto leftFrame = backFrame->leftFrame();
            auto rightFrame = backFrame->rightFrame();

            if ( leftFrame && rightFrame ) {

                auto count = std::max( calcTrackLenght( leftFrame->framePoints() ), calcTrackLenght( rightFrame->framePoints() ) );

                size_t counter = 0;

                for ( auto i = m_frames.rbegin(); i != m_frames.rend() && counter < count; ++i, ++counter ) {
                    auto frame = std::dynamic_pointer_cast< StereoKeyFrame >( *i );
                    if ( frame )
                        list.push_front( frame );
                }

                m_optimizer.adjust( list );

            }

        }

    }

}

bool Map::isRudimental() const
{
    return m_frames.size() <= 1;
}

StereoCameraMatrix Map::backProjectionMatrix() const
{
    for ( auto i = m_frames.rbegin(); i != m_frames.rend(); ++i ) {

        auto keyFrame = std::dynamic_pointer_cast< StereoKeyFrame >( *i );

        if ( keyFrame )
            return keyFrame->projectionMatrix();

    }

    return m_projectionMatrix;
}


bool Map::track( const StampedImage &leftImage, const StampedImage &rightImage )
{    
    static FlowDenseFramePtr keyFrame;

    /*if ( m_denseFlag && m_frames.size() % m_denseStep == 0 )
        denseFrame->processDenseCloud();*/

    if ( m_frames.empty() ) {

        auto frame = FlowDenseFrame::create( shared_from_this() );

        keyFrame = frame;

        frame->setImage( leftImage, rightImage );

        frame->setProjectionMatrix( m_projectionMatrix );
        m_frames.push_back( frame );

        return true;

    }
    else {

        auto frame = FlowStereoFrame::create( shared_from_this() );

        frame->setImage( leftImage, rightImage );

        auto previousFrame = std::dynamic_pointer_cast< FlowStereoFrame >( m_frames.back() );
        auto previousKeyFrame = std::dynamic_pointer_cast< FlowDenseFrame >( m_frames.back() );

        if ( previousKeyFrame ) {

            auto previousLeftFrame = previousKeyFrame->leftFrame();
            auto leftFrame = frame->leftFrame();

            previousLeftFrame->extractPoints();

            std::cout << "Extracted points count: " << previousLeftFrame->flowPointsCount() << std::endl;

            previousKeyFrame->match();
            slam::track( previousLeftFrame, leftFrame );

            std::cout << "Stereo points count: " << previousKeyFrame->stereoPointsCount() << std::endl;

            auto trackedPointCount = previousLeftFrame->trackedPointsCount();

            previousKeyFrame->triangulatePoints();

            std::cout << "Tracked points count: " << trackedPointCount << std::endl;

            previousLeftFrame->cleanMapPoints();

            if ( trackedPointCount < m_minTrackPoints )
                return false;

        }
        else if ( previousFrame ) {

            auto previousLeftFrame = previousFrame->leftFrame();
            auto leftFrame = frame->leftFrame();

            slam::track( previousLeftFrame, leftFrame );

            auto trackedPointsCount = previousLeftFrame->trackedPointsCount();

            previousFrame->cleanMapPoints();

            auto replacedFrame = FinishedStereoFrame::create( shared_from_this() );
            replacedFrame->replaceAndClean( previousFrame );

            m_frames.back() = replacedFrame;

            if ( trackedPointsCount < m_minTrackPoints )
                return false;

        }

        if ( keyFrame ) {

            auto previousLeftFrame = keyFrame->leftFrame();
            auto previousRightFrame = keyFrame->rightFrame();
            auto leftFrame = frame->leftFrame();

            FlowConsecutiveFrame adjacentFrame( previousLeftFrame, leftFrame );

            auto avgDisp = adjacentFrame.averageMapPointsDisplacement();

            auto minTrackInliersRatio = parentWorld()->minTrackInliersRatio();

            if ( avgDisp > parentWorld()->minAdjacentPointsDistance() || adjacentFrame.mapPoints().size() < m_minTrackPoints ) {

                RecoverPoseFrame recoverFrame( previousLeftFrame, leftFrame );

                ProjectionMatrix recoveredPose;

                auto inliersRatio = recoverFrame.recoverPose( &recoveredPose );

                std::cout << "Inliers ratio: " << inliersRatio << std::endl;

                if ( inliersRatio > minTrackInliersRatio /*|| adjacentFrame.mapPoints().size() < m_minTrackPoints*/ ) {

                    auto newKeyFrame = FlowDenseFrame::create( shared_from_this() );

                    newKeyFrame->replace( frame );

                    auto leftFrame = newKeyFrame->leftFrame();
                    auto rightFrame = newKeyFrame->rightFrame();

                    leftFrame->setProjectionMatrix( recoveredPose );

                    rightFrame->setCameraMatrix( previousRightFrame->cameraMatrix() );
                    rightFrame->setRotation( recoveredPose.rotation() );
                    rightFrame->setTranslation( recoveredPose.translation() + baselineVector() );

                    ConsecutiveKeyFrame triangulateFrame( previousLeftFrame, leftFrame );

                    if ( triangulateFrame.distance() > baselineLenght() )
                        triangulateFrame.triangulatePoints();

                    m_frames.push_back( newKeyFrame );

                    auto replacedFrame = FinishedStereoKeyFrame::create( shared_from_this() );
                    replacedFrame->replaceAndClean( keyFrame );

                    auto it = std::find( m_frames.begin(), m_frames.end(), keyFrame );

                    if ( it != m_frames.end() )
                        *it = replacedFrame;

                    keyFrame = newKeyFrame;

                    return true;

                }

            }

        }

        m_frames.push_back( frame );

        return true;

    }

    return false;

}

}

