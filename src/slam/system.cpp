#include "src/common/precompiled.h"

#include "system.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

System::System( const StereoCalibrationDataShort &calibration )
    : m_rectificationProcessor( calibration ),
      m_leftUndistortionProcessor( calibration.leftCameraResults() ),
      m_rightUndistortionProcessor( calibration.rightCameraResults() )
{
    initialize();
}

System::System( const std::string &calibrationFile )
{
    m_rectificationProcessor.loadFile( calibrationFile );

    m_leftUndistortionProcessor.setCalibrationData( m_rectificationProcessor.calibration().leftCameraResults() );
    m_rightUndistortionProcessor.setCalibrationData( m_rectificationProcessor.calibration().rightCameraResults() );

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

std::list< System::WorldPointPtr > &System::worldPoints()
{
    return m_world->worldPoints();
}

const std::list< System::WorldPointPtr > &System::worldPoints() const
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

CvImage System::opticalFlowImage() const
{
    return m_world->opticalFlowImage();
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

    cv::resize( leftRectifiedImage, leftRectifiedImage, cv::Size(), 0.5, 0.5 );
    cv::resize( rightRectifiedImage, rightRectifiedImage, cv::Size(), 0.5, 0.5 );

    return m_world->track( leftRectifiedImage, rightRectifiedImage );

}

}
