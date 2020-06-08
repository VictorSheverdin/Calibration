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

        while ( !isInterruptionRequested() ) {
            m_system->localAdjustment();
            std::this_thread::sleep_for( std::chrono::seconds( 1 ) );

        }

    } );

    auto optimizationThread5s = std::thread( [ & ] {

        while ( !isInterruptionRequested() ) {

            const int frames = 100;

            m_system->adjust( frames );
            std::this_thread::sleep_for( std::chrono::seconds( 5 ) );

        }

    } );

    while( !isInterruptionRequested() )
    {
        if ( m_mutex.tryLock( ) ) {

            if ( !m_leftFrame.empty() && !m_rightFrame.empty() ) {

                CvImage leftRectifiedImage;
                CvImage rightRectifiedImage;
                CvImage leftCroppedImage;
                CvImage rightCroppedImage;

                cv::rectangle( m_leftFrame, cv::Point( 0, 1500 ), cv::Point( 2048, 2048 ), cv::Scalar( 0, 0, 0, 255 ), cv::FILLED );
                cv::rectangle( m_rightFrame, cv::Point( 0, 1500 ), cv::Point( 2048, 2048 ), cv::Scalar( 0, 0, 0, 255 ), cv::FILLED );

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
                    else {

                        /*cv::medianBlur( leftCroppedImage, leftCroppedImage, 3 );
                        cv::medianBlur( rightCroppedImage, rightCroppedImage, 3 );

                        cv::Mat sharpeningKernel = ( cv::Mat_< double >( 3, 3 ) << 0, -1, 0,
                                -1, 5, -1,
                                0, -1, 0 );

                        cv::filter2D( leftCroppedImage, leftCroppedImage, -1, sharpeningKernel );
                        cv::filter2D( rightCroppedImage, rightCroppedImage, -1, sharpeningKernel );*/

                        cv::GaussianBlur( leftCroppedImage, leftCroppedImage, cv::Size( 3, 3 ), 0 );
                        cv::GaussianBlur( rightCroppedImage, rightCroppedImage, cv::Size( 3, 3 ), 0 );

                        m_system->track( leftCroppedImage, rightCroppedImage );
                    }

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

    static std::ofstream file( "/home/victor/coords.txt" );

    auto maps = this->maps();

    for ( auto &i : maps ) {

        auto frames = i->frames();

        for ( auto &j : frames ) {

            cv::Mat leftTranslation = -j->leftFrame()->rotation().t() * j->leftFrame()->translation();

            file << leftTranslation.at< double >( 0, 0 ) << " " << leftTranslation.at< double >( 1, 0 ) << " " << leftTranslation.at< double >( 2, 0 ) << std::endl;

        }

    }

    localOptimizationThread.join();
    optimizationThread5s.join();

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

            auto processedFrame = std::dynamic_pointer_cast< slam::FeatureStereoFrame >( frames.back() );

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

            auto processedFrame = std::dynamic_pointer_cast< slam::FeatureStereoFrame >( frames.back() );

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

            auto processedFrame = std::dynamic_pointer_cast< slam::FeatureStereoFrame >( *(++frames.rbegin()) );

            if ( processedFrame )
                return processedFrame->drawStereoCorrespondences();

        }

    }

    return CvImage();

}
