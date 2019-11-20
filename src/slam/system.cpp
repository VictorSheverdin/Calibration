#include "src/common/precompiled.h"

#include "system.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

System::System( const StereoCalibrationDataShort &calibration )
    : m_rectificationProcessor( calibration )
{
    initialize();
}

System::System( const std::string &calibrationFile )
    : m_rectificationProcessor( calibrationFile )
{
    initialize();
}

void System::initialize()
{
    m_world = World::create( m_rectificationProcessor.calibration() );
}

std::list< System::FramePtr > &System::frames()
{
    return m_world->frames();
}

const std::list< System::FramePtr > &System::frames() const
{
    return m_world->frames();
}

std::vector< System::WorldPointPtr > &System::worldPoints()
{
    return m_world->worldPoints();
}

const std::vector< System::WorldPointPtr > &System::worldPoints() const
{
    return m_world->worldPoints();
}

CvImage System::keyPointsImage() const
{
    return m_world->keyPointsImage();
}

CvImage System::stereoPointsImage() const
{
    return m_world->stereoPointsImage();
}

CvImage System::tracksImage() const
{
    return m_world->tracksImage();
}

bool System::track( const std::string &leftFile, const std::string &rightFile )
{
    CvImage leftImage( leftFile );
    CvImage rightImage( rightFile );

    return track( leftImage, rightImage );

}

bool System::track( const CvImage &leftImage, const CvImage &rightImage )
{
    CvImage leftRectifiedImage;
    CvImage rightRectifiedImage;

    if ( !m_rectificationProcessor.rectify( leftImage, rightImage, &leftRectifiedImage, &rightRectifiedImage ) )
        return false;

    return m_world->track( leftRectifiedImage, rightRectifiedImage );

}

}
