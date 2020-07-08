#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

const double World::m_maxReprojectionError = 1.;
const double World::m_minStereoDisparity = 3.;
const double World::m_minAdjacentPointsDistance = 3.;
const double World::m_minAdjacentCameraMultiplier = 1.;

World::World( const StereoCameraMatrix &cameraMatrix )
{
    initialize( cameraMatrix );

    m_stereoProcessor.setDisparityToDepthMatrix( cameraMatrix.disparityToDepthMatrix() );

    m_stereoProcessor.setPreFilterSize( 15 );
    m_stereoProcessor.setPreFilterCap( 12 );
    m_stereoProcessor.setBlockSize( 17 );
    m_stereoProcessor.setMinDisparity( 0 );
    m_stereoProcessor.setNumDisparities( 256 );
    m_stereoProcessor.setTextureThreshold( 50 );
    m_stereoProcessor.setUniquenessRatio( 20 );
    m_stereoProcessor.setSpeckleWindowSize( 120 );
    m_stereoProcessor.setSpeckleRange( 10 );
    m_stereoProcessor.setDisp12MaxDiff( 0 );
}

void World::initialize( const StereoCameraMatrix &cameraMatrix )
{
    m_trackType = TrackType::FLOW;

    auto flowTracker = new CPUFlowTracker();
    flowTracker->setCount( m_pointsCount );
    m_flowTracker = std::unique_ptr< FlowTracker >( flowTracker );

    m_featureTracker = std::unique_ptr< FeatureTracker >( new SiftTracker() );

    m_startCameraMatrix = cameraMatrix;
}

World::ObjectPtr World::create( const StereoCameraMatrix &cameraMatrix )
{
    return ObjectPtr( new World( cameraMatrix ) );
}

std::list < World::MapPtr > World::maps() const
{
    std::list < World::MapPtr > ret;

    m_mutex.lock();

    ret = m_maps;

    m_mutex.unlock();

    return ret;
}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    auto restoreRotation = this->restoreRotation();
    auto restoreTranslation = this->restoreTranslation();

    if ( m_maps.empty() ) {

        auto startCameraMatrix = m_startCameraMatrix;

        if ( !restoreRotation.empty() && !restoreTranslation.empty() ) {
            startCameraMatrix.rotate( restoreRotation );
            startCameraMatrix.translate( restoreTranslation );

        }

        createMap( startCameraMatrix );

    }

    if ( !m_maps.empty() ) {

        auto result = m_maps.back()->track( leftImage, rightImage );

        if ( !result ) {

            StereoCameraMatrix projectionMatrix;

            if ( m_maps.back()->isRudimental() )
                m_maps.pop_back();

            if ( !restoreRotation.empty() && !restoreTranslation.empty() ) {

                projectionMatrix = m_startCameraMatrix;

                projectionMatrix.rotate( restoreRotation );
                projectionMatrix.translate( restoreTranslation );

            }
            else
                projectionMatrix = m_maps.back()->frames().back()->projectionMatrix();

            createMap( projectionMatrix );

            m_maps.back()->track( leftImage, rightImage );

        }

        return result;

    }

    return false;

}

void World::setRestoreMatrix( const cv::Mat &rotation, const cv::Mat &translation )
{
    m_restoreMatrix = cv::Mat( 3, 4, CV_64F );
    rotation.copyTo( m_restoreMatrix.rowRange( 0, 3 ).colRange( 0, 3 ) );
    translation.copyTo( m_restoreMatrix.rowRange( 0, 3 ).col( 3 ) );
}

const cv::Mat &World::restoreMatrix() const
{
    return m_restoreMatrix;
}

cv::Mat World::restoreRotation() const
{
    return m_restoreMatrix.rowRange( 0, 3 ).colRange( 0, 3 );
}

cv::Mat World::restoreTranslation() const
{
    return m_restoreMatrix.rowRange( 0, 3 ).col( 3 );
}

void World::adjust( const int frames )
{
    if ( !m_maps.empty() )
        m_maps.back()->adjust( frames );
}

void World::localAdjustment()
{
    if ( !m_maps.empty() )
        m_maps.back()->localAdjustment();
}

BMStereoProcessor &World::stereoProcessor()
{
    return m_stereoProcessor;
}

const BMStereoProcessor &World::stereoProcessor() const
{
    return m_stereoProcessor;
}

const std::unique_ptr< FlowTracker > &World::flowTracker() const
{
    return m_flowTracker;
}

const std::unique_ptr<FeatureTracker> &World::featureTracker() const
{
    return m_featureTracker;
}

double World::maxReprojectionError() const
{
    return m_maxReprojectionError;
}

double World::minStereoDisparity() const
{
    return m_minStereoDisparity;
}

double World::minAdjacentPointsDistance() const
{
    return m_minAdjacentPointsDistance;
}

double World::minAdjacentCameraMultiplier() const
{
    return m_minAdjacentCameraMultiplier;
}

void World::createMap( const StereoCameraMatrix &cameraMatrix )
{
    if ( m_trackType == TrackType::FLOW )
        m_maps.push_back( FlowMap::create( cameraMatrix, shared_from_this() ) );
    else if ( m_trackType == TrackType::FEATURES )
        m_maps.push_back( FeatureMap::create( cameraMatrix, shared_from_this() ) );

}

}
