#pragma once

#include "image.h"

#include "VimbaCPP/Include/VimbaCPP.h"

void drawTraceLines( CvImage &image, const unsigned int count );

CvImage resizeTo( const CvImage &image, unsigned int size );

CvImage stackImages( const CvImage &leftImage, const CvImage &rightImage, const double factor = 1.0 );

CvImage makeOverlappedPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );
CvImage makeStraightPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );

CvImage colorizeDisparity( const cv::Mat &disparity );

void checkVimbaStatus( VmbErrorType status, std::string message );

template <typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );

}

template <typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::VimbaSystem &system, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( system.GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );

}

template <typename FeatureT>
FeatureT getVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key )
{
    FeatureT ret;

    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->GetValue( ret ), "Could not get " + key );

    return ret;

}

void vimbaRunCommand( AVT::VmbAPI::VimbaSystem &system, const std::string &key );

void vimbaRunCommand(AVT::VmbAPI::CameraPtr camera, const std::string &key );
