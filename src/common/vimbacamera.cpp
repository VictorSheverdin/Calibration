#include "precompiled.h"

#include "vimbacamera.h"

#include "functions.h"

#include <unistd.h>

// FrameObserver
int FrameObserver::m_currentNumber = 0;
const std::chrono::time_point< std::chrono::system_clock > FrameObserver::m_startTime = std::chrono::system_clock::now();

FrameObserver::FrameObserver( AVT::VmbAPI::CameraPtr pCamera )
    : AVT::VmbAPI::IFrameObserver( pCamera )
{
    inititalize();
}

void FrameObserver::inititalize()
{
    m_number = m_currentNumber++;
}

int64_t FrameObserver::timeFromStart()
{
    return std::chrono::duration_cast< std::chrono::microseconds >( std::chrono::system_clock::now() - m_startTime ).count();
}

void FrameObserver::FrameReceived ( const AVT::VmbAPI::FramePtr pFrame )
{
    VmbFrameStatusType eReceiveStatus ;

    if( VmbErrorSuccess == pFrame->GetReceiveStatus ( eReceiveStatus ) )
    {
        if ( VmbFrameStatusComplete == eReceiveStatus )
        {

            auto time = std::chrono::system_clock::now();

            VmbUchar_t *pImage;
            VmbUint32_t nWidth = 0;
            VmbUint32_t nHeight = 0;

            checkVimbaStatus( pFrame->GetWidth( nWidth ), "FAILED to aquire width of frame!" );
            checkVimbaStatus( pFrame->GetHeight( nHeight ), "FAILED to aquire height of frame!" );
            checkVimbaStatus( pFrame->GetImage( pImage ), "FAILED to acquire image data of frame!" );

            m_framesMutex.lock();
            m_sourceMat = cv::Mat( nHeight, nWidth, CV_8UC1, pImage );
            m_sourceTime = time;
            m_framesMutex.unlock();

            emit receivedFrame();

        }

    }

    m_pCamera->QueueFrame ( pFrame );

}

Frame FrameObserver::getFrame()
{
    Frame res;

    m_framesMutex.lock();

    if ( !m_sourceMat.empty() ) {
        res.setTime( m_sourceTime );
        cv::cvtColor( m_sourceMat, res, cv::COLOR_BayerGB2RGB );
    }

    m_framesMutex.unlock();

    return res;

}

Frame FrameObserver::nearestFrame( const Frame &frame )
{
    return getFrame();
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
    m_camera->StopContinuousImageAcquisition();

    m_camera->Close();

}

void MasterCamera::initialize( const std::string &ip )
{
    auto &vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();

    checkVimbaStatus( vimbaSystem.OpenCameraByID( ip.c_str(), VmbAccessModeFull, m_camera ),
        std::string( "Could not start open camera; ip = " ) + ip );

    setVimbaFeature( m_camera, "PixelFormat", VmbPixelFormatBayerGB8 );

    vimbaRunCommand( m_camera, "GVSPAdjustPacketSize" );

    setVimbaFeature( m_camera, "ExposureTimeAbs", 10000.0 );

    setVimbaFeature( m_camera, "Gain", 13.0 );

    setVimbaFeature( m_camera, "TriggerMode", "Off" );

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

    m_camera->StopContinuousImageAcquisition();

    m_camera->Close();

}

void SlaveCamera::initialize( const std::string &ip )
{
    auto &vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();

    checkVimbaStatus( vimbaSystem.OpenCameraByID( ip.c_str(), VmbAccessModeFull, m_camera ),
        std::string( "Could not start open camera; ip = " ) + ip );

    setVimbaFeature( m_camera, "PixelFormat", VmbPixelFormatBayerGB8 );

    vimbaRunCommand( m_camera, "GVSPAdjustPacketSize" );

    setVimbaFeature( m_camera, "ExposureTimeAbs", 10000.0 );

    setVimbaFeature( m_camera, "Gain", 13.0 );

    setVimbaFeature( m_camera, "TriggerSelector", "FrameStart" );
    setVimbaFeature( m_camera, "TriggerSource", "Line1" );
    setVimbaFeature( m_camera, "TriggerMode", "On" );

    SP_SET( m_frameObserver, new FrameObserver( m_camera ) );

    checkVimbaStatus( SP_ACCESS( m_camera )->StartContinuousImageAcquisition( m_numFrames,  m_frameObserver ), "Can't start image acquisition" );

    connect( m_frameObserver.get(), &FrameObserver::receivedFrame, this, &SlaveCamera::receivedFrame );

}

// StereoCamera
StereoCamera::StereoCamera( const std::string &leftIp, const std::string &rightIp, QObject *parent )
    : QObject( parent ), m_leftCamera( leftIp, parent ), m_rightCamera( rightIp, parent )
{
    initialize();
}

void StereoCamera::initialize()
{
    connect( &m_leftCamera, &SlaveCamera::receivedFrame, this, &StereoCamera::updateFrame );
    connect( &m_rightCamera, &SlaveCamera::receivedFrame, this, &StereoCamera::updateFrame );
}

void StereoCamera::updateFrame()
{
    auto leftFrame = m_leftCamera.getFrame();
    if ( !leftFrame.empty() ) {
        auto rightFrame = m_rightCamera.nearestFrame( leftFrame );

        if ( !rightFrame.empty() ) {

            if ( leftFrame.timeDiff( rightFrame ) < 2000 ) {

                m_framesMutex.lock();
                m_framesQueue.push( StereoFrame( leftFrame, rightFrame ) );
                m_framesMutex.unlock();

                qDebug() << "Difference time:" << leftFrame.timeDiff( rightFrame ) /*<< "+"*/;

                emit receivedFrame();

            }
            /*else {
                qDebug() << "Difference time:" << leftFrame.timeDiff( rightFrame );
            }*/

        }

    }

}

StereoFrame StereoCamera::getFrame()
{
    StereoFrame res;

    m_framesMutex.lock();

    if( !m_framesQueue.empty() ) {
        res = m_framesQueue.back();
    }

    m_framesMutex.unlock();

    return res;

}

bool StereoCamera::empty() const
{
    return m_framesQueue.empty();
}
