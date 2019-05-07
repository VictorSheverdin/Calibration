#include "precompiled.h"

#include "vimbacamera.h"

#include "functions.h"

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

            // Lock the frame queue
            m_framesMutex.lock();
            // Add frame to queue
            m_frame = frame;
            // Unlock frame queue
            m_framesMutex.unlock();

            emit receivedFrame();

        }

    }
    // When you are finished copying the frame , re - queue it

    m_pCamera->QueueFrame ( pFrame );
}

CvImage FrameObserver::getFrame()
{
    // Lock the frame queue
    m_framesMutex.lock();
    // Pop frame from queue
    CvImage res;

//    if( !m_frames.empty() )
//    {
        res = m_frame;
        m_frame = CvImage();
//    }
    // Unlock frame queue
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
    m_camera->Close();
}

void VimbaCamera::initialize( const std::string &ip )
{
    auto &vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();

    checkVimbaStatus( vimbaSystem.OpenCameraByID( ip.c_str(), VmbAccessModeFull, m_camera ),
        std::string( "Could not start open camera; ip = " ) + ip );

    setVimbaFeature( m_camera, "PixelFormat", VmbPixelFormatBgr8 );

    SP_SET( m_frameObserver, new FrameObserver( m_camera ) );

    checkVimbaStatus( SP_ACCESS( m_camera )->StartContinuousImageAcquisition( m_numFrames,  m_frameObserver ), "Can not start image acquisition" );

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
