#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

World::World( const StereoCameraMatrix &cameraMatrix )
{
    initialize( cameraMatrix );

    slam::DenseFrameBase::setDisparityToDepthMatrix( cameraMatrix.disparityToDepthMatrix() );

    slam::DenseFrameBase::setPreFilterSize( 15 );
    slam::DenseFrameBase::setPreFilterCap( 12 );
    slam::DenseFrameBase::setBlockSize( 7 );
    slam::DenseFrameBase::setMinDisparity( 0 );
    slam::DenseFrameBase::setNumDisparities( 256 );
    slam::DenseFrameBase::setTextureThreshold( 350 );
    slam::DenseFrameBase::setUniquenessRatio( 5 );
    slam::DenseFrameBase::setSpeckleWindowSize( 65 );
    slam::DenseFrameBase::setSpeckleRange( 5 );
    slam::DenseFrameBase::setDisp12MaxDiff( 0 );

}

void World::initialize( const StereoCameraMatrix &cameraMatrix )
{
    ProcessedFrame::setMaxFeatures( m_keypointsCount );

    m_startCameraMatrix = cameraMatrix;

}

World::WorldPtr World::create( const StereoCameraMatrix &cameraMatrix )
{
    return WorldPtr( new World( cameraMatrix ) );
}

const std::list < World::MapPtr > &World::maps() const
{
    return m_maps;
}

std::list < World::MapPtr > &World::maps()
{
    return m_maps;
}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    if ( m_maps.empty() )
        m_maps.push_back( Map::create( m_startCameraMatrix, shared_from_this() ) );

    auto result = m_maps.back()->track( leftImage, rightImage );

    if ( !result ) {
        auto lastProjectionMatrix = m_maps.back()->frames().back()->projectionMatrix();

        m_maps.push_back( Map::create( lastProjectionMatrix, shared_from_this() ) );

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

}
