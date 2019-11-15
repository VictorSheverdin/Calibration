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

    // cv::resize( leftRectifiedImage, leftRectifiedImage, cv::Size(), 0.5, 0.5, cv::INTER_AREA );
    // cv::resize( rightRectifiedImage, rightRectifiedImage, cv::Size(), 0.5, 0.5, cv::INTER_AREA );

    return m_world->track( leftRectifiedImage, rightRectifiedImage );

}

}
