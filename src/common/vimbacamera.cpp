#include "precompiled.h"

#include "vimbacamera.h"

#include "functions.h"

#include <unistd.h>

// FrameObserver
FrameObserver::FrameObserver( AVT::VmbAPI::CameraPtr pCamera )
    : AVT::VmbAPI::IFrameObserver( pCamera )
{
}

void FrameObserver::FrameReceived ( const AVT::VmbAPI::FramePtr pFrame )
{
    VmbFrameStatusType eReceiveStatus ;

    if( VmbErrorSuccess == pFrame->GetReceiveStatus ( eReceiveStatus ) )
    {
        if ( VmbFrameStatusComplete == eReceiveStatus )
        {
            CvImage frame;

            VmbUchar_t *pImage;
            VmbUint32_t nWidth = 0;
            VmbUint32_t nHeight = 0;

            checkVimbaStatus( pFrame->GetWidth( nWidth ), "FAILED to aquire width of frame!" );
            checkVimbaStatus( pFrame->GetHeight( nHeight ), "FAILED to aquire height of frame!" );
            checkVimbaStatus( pFrame->GetImage( pImage ), "FAILED to acquire image data of frame!" );

            auto mat = cv::Mat( nHeight, nWidth, CV_8UC3, pImage );

            mat.copyTo( frame );

            m_framesMutex.lock();
            m_framesQueue.push( frame );
            m_framesMutex.unlock();

            emit receivedFrame();

        }

    }

    m_pCamera->QueueFrame ( pFrame );

}

CvImage FrameObserver::getFrame()
{
    // Lock the frame queue
    m_framesMutex.lock();
    // Pop frame from queue
    CvImage res;

    if( !m_framesQueue.empty() )
    {
        res = m_framesQueue.front();
        m_framesQueue.pop();
    }

    m_framesMutex.unlock();

    return res;

}

// VimbaCamera
VimbaCamera::VimbaCamera( const std::string &ip )
{
    initialize( ip );
}

VimbaCamera::~VimbaCamera()
{
    setVimbaFeature( m_camera, "TriggerMode", "Off" );

    m_camera->Close();

}

void VimbaCamera::initialize( const std::string &ip )
{
    auto &vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();

    checkVimbaStatus( vimbaSystem.OpenCameraByID( ip.c_str(), VmbAccessModeFull, m_camera ),
        std::string( "Could not start open camera; ip = " ) + ip );

    setVimbaFeature( m_camera, "PixelFormat", VmbPixelFormatBgr8 );

    setVimbaFeature( m_camera, "TriggerSelector", "FrameStart" );
    setVimbaFeature( m_camera, "TriggerSource", "Action0" );
    setVimbaFeature( m_camera, "TriggerMode", "On" );

    setVimbaFeature( m_camera, "ActionDeviceKey", ACTION_DEVICE_KEY );
    setVimbaFeature( m_camera, "ActionGroupKey", ACTION_GROUP_KEY );
    setVimbaFeature( m_camera, "ActionGroupMask", ACTION_GROUP_MASK );

    SP_SET( m_frameObserver, new FrameObserver( m_camera ) );

    checkVimbaStatus( SP_ACCESS( m_camera )->StartContinuousImageAcquisition( m_numFrames,  m_frameObserver ), "Can't start image acquisition" );

    connect( m_frameObserver.get(), &FrameObserver::receivedFrame, this, &VimbaCamera::receivedFrame );

}

CvImage VimbaCamera::getFrame()
{
    return m_frameObserver->getFrame();
}

void VimbaCamera::setMaxValue( const char* const name )
{
    AVT::VmbAPI::FeaturePtr      feature;

    VmbInt64_t value_min, value_max;

    checkVimbaStatus( SP_ACCESS( m_camera )->GetFeatureByName( name, feature ), "Coldn't get feature" );

    checkVimbaStatus( SP_ACCESS( feature )->GetRange( value_min, value_max ), "Coldn't get range" );

    value_max = ( value_max >> 1 ) << 1;

    checkVimbaStatus( SP_ACCESS( feature )->SetValue ( value_max ), "Coldn't set maximum value" );

}

