#include "src/common/precompiled.h"

#include "system.h"

#include "src/common/functions.h"

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
    cv::namedWindow( "KeyPoints", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "KeyPoints", 800, 600 );
    cv::moveWindow( "KeyPoints", 80, 10 );

    cv::namedWindow( "StereoPoints", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "StereoPoints", 800, 600 );
    cv::moveWindow( "StereoPoints", 900, 10 );

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

    StereoFrame frame( leftRectifiedImage, rightRectifiedImage );
    frame.triangulatePoints();

    auto keyPoints = frame.drawKeyPoints( leftRectifiedImage, rightRectifiedImage );
    cv::imshow( "KeyPoints", keyPoints );

    auto stereoPoints = frame.drawStereoPoints( leftRectifiedImage, rightRectifiedImage );
    cv::imshow( "StereoPoints", stereoPoints );

    cv::waitKey(15);

    return true;

}

}
