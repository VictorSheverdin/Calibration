#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"

#include "frame.h"

#include "world.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

namespace slam {

const double Map::m_minTrackInliersRatio = 0.9;

Map::Map( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
    :  m_leftProjectionMatrix( leftProjectionMatrix ), m_rightProjectionMatrix( rightProjectionMatrix )
{
    initialize();

    m_baselineVector = rightProjectionMatrix.translation() - leftProjectionMatrix.translation();
}

void Map::initialize()
{
    m_previousKeypointsCount = m_goodTrackPoints * 2;
}

Map::MapPtr Map::create( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
{
    return MapPtr( new Map( leftProjectionMatrix, rightProjectionMatrix ) );
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

const ProjectionMatrix &Map::leftProjectionMatrix() const
{
    return m_leftProjectionMatrix;
}

const ProjectionMatrix &Map::rightProjectionMatrix() const
{
    return m_rightProjectionMatrix;
}

const cv::Mat &Map::baselineVector() const
{
    return m_baselineVector;
}

double Map::baselineLenght() const
{
    return cv::norm( m_baselineVector );
}

void Map::multiplicateCameraMatrix( const double value )
{
    m_leftProjectionMatrix.multiplicateCameraMatrix( value );
    m_rightProjectionMatrix.multiplicateCameraMatrix( value );
}

void Map::movePrincipalPoint( const cv::Vec2f &value )
{
    m_leftProjectionMatrix.movePrincipalPoint( value );
    m_rightProjectionMatrix.movePrincipalPoint( value );
}

bool Map::valid() const
{
    return true;
}

bool Map::track( const CvImage &leftImage, const CvImage &rightImage )
{
    auto stereoFrame = ProcessedStereoFrame::create( shared_from_this() );

    stereoFrame->load( leftImage, rightImage );

    stereoFrame->extractKeyPoints();

    if ( m_frames.empty() ) {

        stereoFrame->setProjectionMatrix( m_leftProjectionMatrix, m_rightProjectionMatrix );

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

            auto keypointsCount = m_previousKeypointsCount;

            // previousLeftFrame->triangulatePoints();

            int trackedPointCount;

            double inliersRatio = 0.0;

            auto keyPointsCount = previousStereoFrame->leftKeyPointsCount();

            do {

                do {

                    previousStereoFrame->matchOptical( keypointsCount );
                    consecutiveLeftFrame->trackOptical();

                    trackedPointCount = consecutiveLeftFrame->posePointsCount();

                    m_previousKeypointsCount = keypointsCount;

                    keypointsCount *= 2;

                } while( trackedPointCount < m_goodTrackPoints && m_previousKeypointsCount <= keyPointsCount );

                previousStereoFrame->triangulatePoints();

                inliersRatio = consecutiveLeftFrame->recoverPose();

                std::cout << "! " << m_previousKeypointsCount << " " << trackedPointCount << " " << inliersRatio << std::endl;

            } while ( inliersRatio < m_minTrackInliersRatio && m_previousKeypointsCount <= keyPointsCount );

            std::cout << "Inliers: " << inliersRatio << std::endl;

            if ( trackedPointCount < m_minTrackPoints )
                return false;

            if ( trackedPointCount > m_overTrackPoints )
                m_previousKeypointsCount /= 2;

            nextRightFrame->setCameraMatrix( previousRightFrame->cameraMatrix() );
            nextRightFrame->setRotation( nextLeftFrame->rotation() );
            nextRightFrame->setTranslation( nextLeftFrame->translation() + baselineVector() );

            previousStereoFrame->cleanMapPoints();

            auto replacedFrame = StereoFrame::create();

            replacedFrame->replaceAndClean( previousStereoFrame );

            m_frames.back() = replacedFrame;

            m_frames.push_back( stereoFrame );

        }

    }

    return true;

}

}

