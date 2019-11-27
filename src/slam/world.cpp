#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

World::World( const StereoCalibrationDataShort &calibration )
{
    m_rectificationProcessor.setCalibrationData( calibration );
    m_leftUndistortionProcessor.setCalibrationData( calibration.leftCameraResults() );
    m_rightUndistortionProcessor.setCalibrationData( calibration.rightCameraResults() );

    initialize();
}

World::World( const std::string &calibrationFile )
{
    m_rectificationProcessor.loadFile( calibrationFile );

    m_leftUndistortionProcessor.setCalibrationData( m_rectificationProcessor.calibration().leftCameraResults() );
    m_rightUndistortionProcessor.setCalibrationData( m_rectificationProcessor.calibration().rightCameraResults() );

    initialize();
}

void World::initialize()
{
    m_map = Map::create( m_rectificationProcessor.calibration() );
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

    cv::resize( leftCroppedImage, leftCroppedImage, cv::Size(), 0.5, 0.5 );
    cv::resize( rightCroppedImage, rightCroppedImage, cv::Size(), 0.5, 0.5 );

    return m_map->track( leftCroppedImage, rightCroppedImage );

}

}
