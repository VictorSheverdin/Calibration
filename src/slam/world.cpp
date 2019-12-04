#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

World::World( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
    : m_leftProjectionMatrix( leftProjectionMatrix ), m_rightProjectionMatrix( rightProjectionMatrix )
{
    initialize();

    m_baselineVector = rightProjectionMatrix.translation() - leftProjectionMatrix.translation();

}

void World::initialize()
{
}

World::WorldPtr World::create( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
{
    return WorldPtr( new World( leftProjectionMatrix, rightProjectionMatrix ) );
}

std::list< World::FramePtr > &World::frames()
{
    return m_map->frames();
}

const std::list< World::FramePtr > &World::frames() const
{
    return m_map->frames();
}

std::set< World::MapPointPtr > &World::mapPoints()
{
    return m_map->mapPoints();
}

const std::set< World::MapPointPtr > &World::mapPoints() const
{
    return m_map->mapPoints();
}

CvImage World::keyPointsImage() const
{
    return m_map->keyPointsImage();
}

CvImage World::stereoPointsImage() const
{
    return m_map->stereoPointsImage();
}

CvImage World::tracksImage() const
{
    return m_map->tracksImage();
}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    return m_map->track( leftImage, rightImage );
}

const ProjectionMatrix &World::leftProjectionMatrix() const
{
    return m_leftProjectionMatrix;
}

const ProjectionMatrix &World::rightProjectionMatrix() const
{
    return m_rightProjectionMatrix;
}

void World::multiplicateCameraMatrix( const double value )
{
    m_leftProjectionMatrix.multiplicateCameraMatrix( value );
    m_rightProjectionMatrix.multiplicateCameraMatrix( value );

}

void World::movePrincipalPoint( const cv::Vec2f &value )
{
    m_leftProjectionMatrix.movePrincipalPoint( value );
    m_rightProjectionMatrix.movePrincipalPoint( value );

}

void World::createMap()
{
    m_map = Map::create( shared_from_this() );
}

const cv::Mat &World::baselineVector() const
{
    return m_baselineVector;
}

double World::baselineLenght() const
{
    return cv::norm( m_baselineVector );
}

}
