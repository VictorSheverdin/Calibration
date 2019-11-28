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
    m_scaleFactor = 1.0;

    m_rectificationProcessor.loadFile( calibrationFile );

    initialize();
}

void World::initialize()
{
}

World::WorldPtr World::create( const StereoCalibrationDataShort &calibration )
{
    return WorldPtr( new World( calibration ) );
}

World::WorldPtr World::create( const std::string &calibrationFile )
{
    return WorldPtr( new World( calibrationFile ) );
}

std::list< World::FramePtr > &World::frames()
{
    return m_map->frames();
}

const std::list< World::FramePtr > &World::frames() const
{
    return m_map->frames();
}

std::list< World::MapPointPtr > &World::mapPoints()
{
    return m_map->mapPoints();
}

const std::list< World::MapPointPtr > &World::mapPoints() const
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

bool World::track( const std::string &leftFile, const std::string &rightFile )
{
    CvImage leftImage( leftFile );
    CvImage rightImage( rightFile );

    return track( leftImage, rightImage );

}

bool World::track( const CvImage &leftImage, const CvImage &rightImage )
{
    CvImage leftRectifiedImage;
    CvImage rightRectifiedImage;

    CvImage leftCroppedImage;
    CvImage rightCroppedImage;

    if ( !m_rectificationProcessor.rectify( leftImage, rightImage, &leftRectifiedImage, &rightRectifiedImage ) )
        return false;

    if ( !m_rectificationProcessor.crop( leftRectifiedImage, rightRectifiedImage, &leftCroppedImage, &rightCroppedImage ) )
        return false;

    return m_map->track( leftCroppedImage, rightCroppedImage );

}

const StereoCalibrationDataShort &World::calibration() const
{
    return m_rectificationProcessor.calibration();
}

void World::setScaleFactor( const double value )
{
    m_scaleFactor = value;
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
