#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

World::World( const StereoCameraMatrix &cameraMatrix )
{
    initialize();

    m_maps.push_back( Map::create( cameraMatrix ) );

    slam::DenseFrameBase::setDisparityToDepthMatrix( cameraMatrix.disparityToDepthMatrix() );

    slam::DenseFrameBase::setPreFilterSize( 15 );
    slam::DenseFrameBase::setPreFilterCap( 12 );
    slam::DenseFrameBase::setBlockSize( 7 );
    slam::DenseFrameBase::setMinDisparity( 0 );
    slam::DenseFrameBase::setNumDisparities( 256 );
    slam::DenseFrameBase::setTextureThreshold( 350 );
    slam::DenseFrameBase::setUniquenessRatio( 10 );
    slam::DenseFrameBase::setSpeckleWindowSize( 65 );
    slam::DenseFrameBase::setSpeckleRange( 5 );
    slam::DenseFrameBase::setDisp12MaxDiff( 0 );

}

void World::initialize()
{
    ProcessedFrame::setMaxFeatures( m_keypointsCount );
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
    return m_maps.back()->track( leftImage, rightImage );
}

void World::adjust( const int frames )
{
    return m_maps.back()->adjust( frames );
}

void World::localAdjustment()
{
    return m_maps.back()->localAdjustment();
}

}
