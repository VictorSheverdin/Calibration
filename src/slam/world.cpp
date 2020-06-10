#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

const double World::m_maxReprojectionError = 2.;
const double World::m_minStereoXDisparity = 15.;
const double World::m_maxStereoYDisparity = 3.;
const double World::m_minAdjacentPointsDistance = 15.;
const double World::m_minAdjacentCameraMultiplier = 2.;

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

    auto tracker = new CPUOpticalTracker();
    tracker->setMaxFeatures( m_keypointsCount );

    m_featureTracker = std::unique_ptr< FeatureTracker >( tracker );

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
    if ( m_maps.empty() )
        createMap( m_startCameraMatrix );

    auto result = m_maps.back()->track( leftImage, rightImage );

    if ( !result ) {

        auto lastProjectionMatrix = m_maps.back()->frames().back()->projectionMatrix();

        if ( m_maps.back()->isRudimental() )
            m_maps.pop_back();

        createMap( lastProjectionMatrix );

        m_maps.back()->track( leftImage, rightImage );

    }

    return result;

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

GPUFlowTracker &World::flowTracker()
{
    return m_flowTracker;
}

const GPUFlowTracker &World::flowTracker() const
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

double World::minStereoXDisparity() const
{
    return m_minStereoXDisparity;
}

double World::maxStereoYDisparity() const
{
    return m_maxStereoYDisparity;
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
