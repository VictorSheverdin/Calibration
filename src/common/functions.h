#include "image.h"

#include "VimbaCPP/Include/VimbaCPP.h"

void drawTraceLines( CvImage &image, const unsigned int count );

CvImage resizeTo( const CvImage &image, unsigned int size );

void checkVimbaStatus( VmbErrorType status, std::string message );

template<typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;
    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key);
    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );

}
