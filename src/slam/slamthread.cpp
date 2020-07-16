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
    m_mutex.lock();

    m_leftFrame = leftImage;
    m_rightFrame = rightImage;

    m_mutex.unlock();

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
        if ( m_mutex.tryLock( ) ) {

            if ( !m_leftFrame.empty() && !m_rightFrame.empty() ) {

                CvImage leftRectifiedImage;
                CvImage rightRectifiedImage;
                CvImage leftCroppedImage;
                CvImage rightCroppedImage;

                // cv::rectangle( m_leftFrame, cv::Point( 0, 1500 ), cv::Point( 2048, 2048 ), cv::Scalar( 0, 0, 0, 255 ), cv::FILLED );
                // cv::rectangle( m_rightFrame, cv::Point( 0, 1500 ), cv::Point( 2048, 2048 ), cv::Scalar( 0, 0, 0, 255 ), cv::FILLED );

                TicToc time;

                // leftCroppedImage = m_leftUndistortionProcessor.undistort( m_leftFrame );
                // rightCroppedImage = m_rightUndistortionProcessor.undistort( m_rightFrame );

                if ( m_rectificationProcessor.rectify( m_leftFrame, m_rightFrame, &leftRectifiedImage, &rightRectifiedImage )
                            && m_rectificationProcessor.crop( leftRectifiedImage, rightRectifiedImage, &leftCroppedImage, &rightCroppedImage ) ) {

                    CvImage leftProcImage;
                    CvImage rightProcImage;

                    if ( std::abs( m_scaleFactor - 1.0 ) > DOUBLE_EPS ) {
                        cv::resize( leftCroppedImage, leftProcImage, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_AREA );
                        cv::resize( rightCroppedImage, rightProcImage, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_AREA );

                    }
                    else {
                        leftProcImage = leftCroppedImage;
                        rightProcImage = rightCroppedImage;

                    }

                    /*cv::GaussianBlur( leftProcImage, leftProcImage, cv::Size( 3, 3 ), 0 );
                    cv::GaussianBlur( rightProcImage, rightProcImage, cv::Size( 3, 3 ), 0 );*/

                    m_system->track( leftProcImage, rightProcImage );

                    emit updateSignal();

                }

                time.report();

                m_leftFrame.release();
                m_rightFrame.release();

            }

            m_mutex.unlock();

        }

        std::this_thread::sleep_for( std::chrono::microseconds( 1 ) );

    }

    // optimizationThread.join();

}

std::list< SlamThread::MapPtr > SlamThread::maps() const
{
    return m_system->maps();
}

CvImage SlamThread::pointsImage() const
{
    auto maps = m_system->maps();

    if ( !maps.empty() ) {

        auto frames = maps.back()->frames();

        if ( !frames.empty() ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( frames.back() );

            if ( processedFrame )
                return processedFrame->drawExtractedPoints();

        }

    }

    return CvImage();

}

CvImage SlamThread::tracksImage() const
{
    auto maps = m_system->maps();

    if ( !maps.empty() ) {

        auto frames = maps.back()->frames();

        if ( !frames.empty() ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( frames.back() );

            if ( processedFrame )
                return processedFrame->leftFrame()->drawTracks();

        }

    }

    return CvImage();

}

CvImage SlamThread::stereoImage() const
{
    auto maps = m_system->maps();

    if ( !maps.empty() ) {

        auto frames = maps.back()->frames();

        if ( frames.size() > 1 ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( *(++frames.rbegin()) );

            if ( processedFrame )
                return processedFrame->drawStereoCorrespondences();

        }

    }

    return CvImage();

}
