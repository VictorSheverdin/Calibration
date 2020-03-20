#include "src/common/precompiled.h"

#include "slamthread.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

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

    auto cropRect = calibration.cropRect();
    auto principal = cv::Vec2f( -cropRect.x, -cropRect.y );

    auto projectionMatrix = calibration.projectionMatrix();

    projectionMatrix.movePrincipalPoint( principal );
    projectionMatrix.multiplicateCameraMatrix( m_scaleFactor );

    m_system = slam::World::create( projectionMatrix );

}

void SlamThread::process( const CvImage leftImage, const CvImage rightImage )
{
    m_mutex.lock();

    m_leftFrame = leftImage;
    m_rightFrame = rightImage;

    m_mutex.unlock();

}

void SlamThread::run()
{
    auto localOptimizationThread = std::thread( [ & ] {

        while ( true ) {
            m_system->localAdjustment();
            std::this_thread::sleep_for( std::chrono::seconds( 1 ) );

        }

    } );

    localOptimizationThread.detach();

    auto optimizationThread5s = std::thread( [ & ] {

        while ( true ) {

            const int frames = 100;

            m_system->adjust( frames );
            std::this_thread::sleep_for( std::chrono::seconds( 5 ) );

        }

    } );

    optimizationThread5s.detach();

    while( true )
    {
        if ( m_mutex.tryLock( ) ) {

            if ( !m_leftFrame.empty() && !m_rightFrame.empty() ) {

                CvImage leftRectifiedImage;
                CvImage rightRectifiedImage;
                CvImage leftCroppedImage;
                CvImage rightCroppedImage;

                if ( m_rectificationProcessor.rectify( m_leftFrame, m_rightFrame, &leftRectifiedImage, &rightRectifiedImage )
                            && m_rectificationProcessor.crop( leftRectifiedImage, rightRectifiedImage, &leftCroppedImage, &rightCroppedImage ) ) {

                    auto time = std::chrono::system_clock::now();

                    if ( std::abs( m_scaleFactor - 1.0 ) > DOUBLE_EPS ) {
                        CvImage leftResizedImage;
                        CvImage rightResizedImage;

                        cv::resize( leftCroppedImage, leftResizedImage, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_CUBIC );
                        cv::resize( rightCroppedImage, rightResizedImage, cv::Size(), m_scaleFactor, m_scaleFactor, cv::INTER_CUBIC );

                        m_system->track( leftResizedImage, rightResizedImage );
                    }
                    else
                        m_system->track( leftCroppedImage, rightCroppedImage );


                    std::cout << std::chrono::duration_cast< std::chrono::microseconds >( std::chrono::system_clock::now() - time ).count()  / 1.e6 << " sec" << std::endl;

                    emit updateSignal();

                }

                m_leftFrame.release();
                m_rightFrame.release();

            }

            m_mutex.unlock();

        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );

    }

}

const std::list< SlamThread::MapPtr > &SlamThread::maps() const
{
    return m_system->maps();
}

CvImage SlamThread::pointsImage() const
{
    if ( !m_system->maps().empty() ) {

        if ( !m_system->maps().back()->frames().empty() ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( m_system->maps().back()->frames().back() );

            if ( processedFrame )
                return processedFrame->drawPoints();

        }

    }

    return CvImage();

}

CvImage SlamThread::tracksImage() const
{
    if ( !m_system->maps().empty() ) {

        if ( !m_system->maps().back()->frames().empty() ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( m_system->maps().back()->frames().back() );

            if ( processedFrame )
                return processedFrame->leftFrame()->drawTracks();

        }

    }

    return CvImage();

}

CvImage SlamThread::stereoImage() const
{
    if ( !m_system->maps().empty() ) {

        if ( m_system->maps().back()->frames().size() > 1 ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( *(++m_system->maps().back()->frames().rbegin()) );

            if ( processedFrame )
                return processedFrame->drawStereoCorrespondences();

        }

    }

    return CvImage();

}
