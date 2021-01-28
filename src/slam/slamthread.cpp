#include "src/common/precompiled.h"

#include "slamthread.h"

#include "src/common/defs.h"
#include "src/common/functions.h"
#include "src/common/tictoc.h"

#include "world.h"
#include "mappoint.h"
#include "frame.h"

#include <thread>

// SlamThread
SlamThread::SlamThread(const StereoCalibrationDataShort &calibration, QObject *parent )
    : QThread( parent )
{
    initialize( calibration );
}

void SlamThread::initialize( const StereoCalibrationDataShort &calibration )
{
    m_scaleFactor = 1.0;

    m_rectificationProcessor.setCalibrationData( calibration );
    m_leftUndistortionProcessor.setCalibrationData( calibration.leftCameraResults() );
    m_rightUndistortionProcessor.setCalibrationData( calibration.rightCameraResults() );

    auto cropRect = calibration.cropRect();
    auto principal = cv::Vec2f( -cropRect.x, -cropRect.y );

    auto projectionMatrix = calibration.projectionMatrix();

    projectionMatrix.movePrincipalPoint( principal );
    projectionMatrix.multiplicateCameraMatrix( m_scaleFactor );

    m_system = slam::World::create( projectionMatrix );

}

void SlamThread::process( const StampedImage leftImage, const StampedImage rightImage )
{
    m_framesMutex.lock();

    m_leftFrame = leftImage;
    m_rightFrame = rightImage;

    m_framesMutex.unlock();

}

std::shared_ptr< slam::World > SlamThread::system() const
{
    return m_system;
}

CvImage SlamThread::pointsImage() const
{
    m_systemMutex.lock();

    auto ret = m_system->pointsImage();

    m_systemMutex.unlock();

    return ret;
}

CvImage SlamThread::tracksImage() const
{
    m_systemMutex.lock();

    auto ret = m_system->tracksImage();

    m_systemMutex.unlock();

    return ret;
}

CvImage SlamThread::stereoImage() const
{
    m_systemMutex.lock();

    auto ret = m_system->stereoImage();

    m_systemMutex.unlock();

    return ret;
}

std::list< StereoCameraMatrix > SlamThread::path() const
{
    m_systemMutex.lock();

    auto ret = m_system->path();

    m_systemMutex.unlock();

    return ret;

}

std::vector< ColorPoint3d > SlamThread::sparseCloud() const
{
    m_systemMutex.lock();

    auto ret = m_system->sparseCloud();

    m_systemMutex.unlock();

    return ret;

}

void SlamThread::run()
{
   /*auto optimizationThread = std::thread( [ & ] {

        while ( !isInterruptionRequested() ) {

            const int frames = 100;

            m_system->adjust( frames );
            std::this_thread::sleep_for( std::chrono::seconds( 5 ) );

        }

    } );*/

    while( !isInterruptionRequested() )
    {
        m_framesMutex.lock();

        auto leftFrame = m_leftFrame;
        auto rightFrame = m_rightFrame;

        m_framesMutex.unlock();

        if ( !leftFrame.empty() && !rightFrame.empty() ) {

            CvImage leftRectifiedImage;
            CvImage rightRectifiedImage;
            CvImage leftCroppedImage;
            CvImage rightCroppedImage;

            // cv::rectangle( leftFrame, cv::Point( 0, 1500 ), cv::Point( 2048, 2048 ), cv::Scalar( 0, 0, 0, 255 ), cv::FILLED );
            // cv::rectangle( rightFrame, cv::Point( 0, 1500 ), cv::Point( 2048, 2048 ), cv::Scalar( 0, 0, 0, 255 ), cv::FILLED );

            TicToc time;

            // leftCroppedImage = m_leftUndistortionProcessor.undistort( leftFrame );
            // rightCroppedImage = m_rightUndistortionProcessor.undistort( rightFrame );

            if ( m_rectificationProcessor.rectify( leftFrame, rightFrame, &leftRectifiedImage, &rightRectifiedImage )
                        && m_rectificationProcessor.crop( leftRectifiedImage, rightRectifiedImage, &leftCroppedImage, &rightCroppedImage ) ) {

                CvImage leftProcImage;
                CvImage rightProcImage;

                //cv::GaussianBlur( leftCroppedImage, leftCroppedImage, cv::Size( 3, 3 ), 0 );
                //cv::GaussianBlur( rightCroppedImage, rightCroppedImage, cv::Size( 3, 3 ), 0 );

                if ( std::abs( m_scaleFactor - 1.0 ) > DOUBLE_EPS ) {
                    cv::resize( leftCroppedImage, leftProcImage, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_LANCZOS4 );
                    cv::resize( rightCroppedImage, rightProcImage, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_LANCZOS4 );

                }
                else {
                    leftProcImage = leftCroppedImage;
                    rightProcImage = rightCroppedImage;

                }

                m_systemMutex.lock();

                m_system->track( leftProcImage, rightProcImage );

                m_systemMutex.unlock();

                emit updateSignal();

            }

            time.report();

            m_leftFrame.release();
            m_rightFrame.release();

        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );

    }

    // optimizationThread.join();

}
