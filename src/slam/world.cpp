#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

World::World( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )    
{
    initialize();

    m_maps.push_back( Map::create( leftProjectionMatrix, rightProjectionMatrix ) );
}

void World::initialize()
{
    ProcessedStereoFrame::setMaxFeatures( m_keypointsCount );
}

World::WorldPtr World::create( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
{
    return WorldPtr( new World( leftProjectionMatrix, rightProjectionMatrix ) );
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
    // if ( !m_maps.back()->valid() )

    return m_maps.back()->track( leftImage, rightImage );

}

void World::multiplicateCameraMatrix( const double value )
{
    m_maps.back()->multiplicateCameraMatrix( value );
}

void World::movePrincipalPoint( const cv::Vec2f &value )
{
    m_maps.back()->movePrincipalPoint( value );
}

}
