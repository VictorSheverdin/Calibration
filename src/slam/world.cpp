#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

World::World( const StereoCalibrationDataShort &calibration )
{
    m_rectificationProcessor.setCalibrationData( calibration );

    initialize();
}

World::World( const std::string &calibrationFile )
{
    m_rectificationProcessor.loadFile( calibrationFile );

    initialize();
}

World::World( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
    : m_leftProjectionMatrix( leftProjectionMatrix ), m_rightProjectionMatrix( rightProjectionMatrix )
{
    initialize();
}

void World::initialize()
{
    m_scaleFactor = 1.0;
}

World::WorldPtr World::create( const StereoCalibrationDataShort &calibration )
{
    return WorldPtr( new World( calibration ) );
}

World::WorldPtr World::create( const std::string &calibrationFile )
{
    return WorldPtr( new World( calibrationFile ) );
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

const StereoCalibrationDataShort &World::calibration() const
{
    return m_rectificationProcessor.calibration();
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
    m_scaleFactor = value;

    m_leftProjectionMatrix.multiplicateCameraMatrix( value );
    m_rightProjectionMatrix.multiplicateCameraMatrix( value );

}

void World::movePrincipalPoint( const cv::Vec2f &value )
{
    m_leftProjectionMatrix.movePrincipalPoint( value );
    m_rightProjectionMatrix.movePrincipalPoint( value );

}

double World::scaleFactor() const
{
    return m_scaleFactor;
}

void World::createMap()
{
    m_map = Map::create( shared_from_this() );
}

}
