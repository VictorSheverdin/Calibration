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
