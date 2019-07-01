#include "precompiled.h"

#include "vimbacamera.h"

#include "functions.h"

#include <unistd.h>

// FrameObserver
int FrameObserver::m_currentNumber = 0;

FrameObserver::FrameObserver( AVT::VmbAPI::CameraPtr pCamera )
    : AVT::VmbAPI::IFrameObserver( pCamera )
{
    inititalize();
}

void FrameObserver::inititalize()
{
    m_number = m_currentNumber++;
}

void FrameObserver::FrameReceived ( const AVT::VmbAPI::FramePtr pFrame )
{
    VmbFrameStatusType eReceiveStatus ;

    if( VmbErrorSuccess == pFrame->GetReceiveStatus ( eReceiveStatus ) )
    {
        if ( VmbFrameStatusComplete == eReceiveStatus )
        {
            Frame frame;

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

Frame FrameObserver::getFrame()
{
    m_framesMutex.lock();

    Frame res;

    if ( !m_framesQueue.empty() )
        res = m_framesQueue.back();

    m_framesMutex.unlock();

    return res;

}

Frame FrameObserver::nearestFrame( const Frame &frame )
{
    Frame res;

    if ( !frame.empty() ) {

        auto time = frame.time();

        m_framesMutex.lock();

        if ( !m_framesQueue.empty() ) {
            res = m_framesQueue.back();
            auto resTime = abs( std::chrono::duration_cast<std::chrono::milliseconds>( time - res.time() ).count() );

            auto i = m_framesQueue.rbegin();
            ++i;

            for ( ; i != m_framesQueue.rend(); ++i ) {
                auto curTime = abs( std::chrono::duration_cast<std::chrono::milliseconds>( time - i->time() ).count() );
                if ( curTime < resTime ) {
                    resTime = curTime;
                    res = *i;
                }
            }

        }

        m_framesMutex.unlock();
    }

    return res;
}

// CameraBase
CameraBase::CameraBase( QObject *parent )
    : QObject( parent )
{
    initialize();
}

void CameraBase::initialize()
{
}

Frame CameraBase::getFrame()
{
    return m_frameObserver->getFrame();
}

Frame CameraBase::nearestFrame( const Frame &frame )
{
    return m_frameObserver->nearestFrame( frame );
}

void CameraBase::setMaxValue( const char* const name )
{
    AVT::VmbAPI::FeaturePtr      feature;

    VmbInt64_t value_min, value_max;

    checkVimbaStatus( SP_ACCESS( m_camera )->GetFeatureByName( name, feature ), "Coldn't get feature" );

    checkVimbaStatus( SP_ACCESS( feature )->GetRange( value_min, value_max ), "Coldn't get range" );

    value_max = ( value_max >> 1 ) << 1;

    checkVimbaStatus( SP_ACCESS( feature )->SetValue ( value_max ), "Coldn't set maximum value" );

}

// MasterCamera
MasterCamera::MasterCamera( const std::string &ip, QObject *parent )
    : CameraBase( parent )
{
    initialize( ip );
}

MasterCamera::~MasterCamera()
{
    m_camera->Close();

}

void MasterCamera::initialize( const std::string &ip )
{
    auto &vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();

    checkVimbaStatus( vimbaSystem.OpenCameraByID( ip.c_str(), VmbAccessModeFull, m_camera ),
        std::string( "Could not start open camera; ip = " ) + ip );

    setVimbaFeature( m_camera, "PixelFormat", VmbPixelFormatBgr8 );

    setVimbaFeature( m_camera, "TriggerMode", "Off" );
    setVimbaFeature( m_camera, "SyncOutSelector", "SyncOut1" );
    setVimbaFeature( m_camera, "SyncOutSource", "Imaging" );

    SP_SET( m_frameObserver, new FrameObserver( m_camera ) );

    checkVimbaStatus( SP_ACCESS( m_camera )->StartContinuousImageAcquisition( m_numFrames,  m_frameObserver ), "Can't start image acquisition" );

    connect( m_frameObserver.get(), &FrameObserver::receivedFrame, this, &MasterCamera::receivedFrame );

    m_camera->StartCapture();

}

// MasterCamera
SlaveCamera::SlaveCamera( const std::string &ip, QObject *parent )
    : CameraBase( parent )
{
    initialize( ip );
}

SlaveCamera::~SlaveCamera()
{
    setVimbaFeature( m_camera, "TriggerMode", "Off" );

    m_camera->Close();

}

void SlaveCamera::initialize( const std::string &ip )
{
    auto &vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();

    checkVimbaStatus( vimbaSystem.OpenCameraByID( ip.c_str(), VmbAccessModeFull, m_camera ),
        std::string( "Could not start open camera; ip = " ) + ip );

    setVimbaFeature( m_camera, "PixelFormat", VmbPixelFormatBgr8 );

    setVimbaFeature( m_camera, "TriggerSelector", "FrameStart" );
    setVimbaFeature( m_camera, "TriggerSource", "Line1" );
    setVimbaFeature( m_camera, "TriggerMode", "On" );

    SP_SET( m_frameObserver, new FrameObserver( m_camera ) );

    checkVimbaStatus( SP_ACCESS( m_camera )->StartContinuousImageAcquisition( m_numFrames,  m_frameObserver ), "Can't start image acquisition" );

    connect( m_frameObserver.get(), &FrameObserver::receivedFrame, this, &MasterCamera::receivedFrame );

}

// StereoCamera
StereoCamera::StereoCamera( const std::string &leftIp, const std::string &rightIp, QObject *parent )
    : QObject( parent ), m_leftCamera( leftIp, parent ), m_rightCamera( rightIp, parent )
{
    initialize();
}

void StereoCamera::initialize()
{
    connect( &m_rightCamera, &SlaveCamera::receivedFrame, this, &StereoCamera::updateFrame );
}

void StereoCamera::updateFrame()
{
    auto leftFrame = m_leftCamera.getFrame();
    if ( !leftFrame.empty() ) {
        auto rightFrame = m_rightCamera.nearestFrame( leftFrame );

        if ( !rightFrame.empty() ) {

            m_framesMutex.lock();
            m_framesQueue.push( StereoFrame( leftFrame, rightFrame ) );
            m_framesMutex.unlock();

            emit receivedFrame();

        }

    }

}

StereoFrame StereoCamera::getFrame()
{
    // Lock the frame queue
    m_framesMutex.lock();
    // Pop frame from queue
    StereoFrame res;

    if( !m_framesQueue.empty() ) {
        res = m_framesQueue.back();
        qDebug() << res.timeDiff();
    }

    m_framesMutex.unlock();

    return res;

}

bool StereoCamera::empty() const
{
    return m_framesQueue.empty();
}
