#include "src/common/precompiled.h"

#include "stereoresultprocessor.h"

#include "src/common/functions.h"

// StereoResult
StereoResult::StereoResult()
{
    initialize();
}

void StereoResult::initialize()
{
}

void StereoResult::setPreviewImage( const CvImage &value )
{
    m_previewImage = value;
}

void StereoResult::setDisparity( const cv::Mat &value )
{
    m_disparity = value;
    m_colorizedDisparity = colorizeDisparity( m_disparity );
}

void StereoResult::setPoints( const cv::Mat &value )
{
    m_points = value;
}

void StereoResult::setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &value )
{
    m_pointCloud = value;
}

const CvImage &StereoResult::previewImage() const
{
    return m_previewImage;
}

const cv::Mat &StereoResult::disparity() const
{
    return m_disparity;
}

const CvImage &StereoResult::colorizedDisparity() const
{
    return m_colorizedDisparity;
}

const cv::Mat &StereoResult::points() const
{
    return m_points;
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr StereoResult::pointCloud() const
{
    return m_pointCloud;
}

const std::chrono::time_point< std::chrono::system_clock > &StereoResult::time() const
{
    return m_frame.time();
}

void StereoResult::setFrame( const StereoFrame &frame )
{
    m_frame = frame;
}

const StereoFrame &StereoResult::frame() const
{
    return m_frame;
}

// StereoResultProcessor
StereoResultProcessor::StereoResultProcessor()
{
}

StereoResultProcessor::StereoResultProcessor( const std::shared_ptr< DisparityProcessorBase > &proc )
    : StereoProcessor( proc )
{
}

void StereoResultProcessor::setCalibration( const StereoCalibrationDataShort &data )
{
    StereoProcessor::setCalibration( data );
    m_rectificationProcessor.setCalibrationData( data );
}

bool StereoResultProcessor::loadYaml( const std::string &fileName )
{
    bool ret = StereoProcessor::loadYaml( fileName );
    m_rectificationProcessor.setCalibrationData( StereoProcessor::calibration() );

    return ret;
}

StereoResult StereoResultProcessor::process( const StereoFrame &frame )
{
    StereoResult ret;

    ret.setFrame( frame );

    if ( !frame.empty() ) {

        auto leftFrame = frame.leftFrame();
        auto rightFrame = frame.rightFrame();

        if ( m_rectificationProcessor.isValid() ) {

            CvImage leftRectifiedFrame;
            CvImage rightRectifiedFrame;

            CvImage leftCroppedFrame;
            CvImage rightCroppedFrame;

            m_rectificationProcessor.rectify( leftFrame, rightFrame, &leftRectifiedFrame, &rightRectifiedFrame );
            m_rectificationProcessor.crop( leftRectifiedFrame, rightRectifiedFrame, &leftCroppedFrame, &rightCroppedFrame );

            auto previewImage = stackImages( leftCroppedFrame, rightCroppedFrame );
            drawTraceLines( previewImage, 20 );

            ret.setPreviewImage( previewImage );

            if ( m_disparityProcessor ) {

                auto disparity = processDisparity( leftCroppedFrame, rightCroppedFrame );
                ret.setDisparity( disparity );

                cv::Mat points = reprojectPoints( disparity );
                ret.setPoints( points );

                auto pointCloud = producePointCloud( points, leftCroppedFrame );
                ret.setPointCloud( pointCloud );

            }

        }

    }

    return ret;

}

// ProcessorThread
ProcessorThread::ProcessorThread( QObject *parent )
    : QThread( parent )
{
    initialize();
}

void ProcessorThread::initialize()
{
}

bool ProcessorThread::process(const StereoFrame &frame )
{
    bool res = false;

    if ( m_processMutex.tryLock() ) {
        if ( !isRunning() ) {
            m_frame = frame;
            start();
            res = true;
        }

        m_processMutex.unlock();
    }

    return res;

}

void ProcessorThread::setProcessor( const std::shared_ptr< StereoResultProcessor > processor )
{
    m_processor = processor;
}

StereoResult ProcessorThread::result()
{
    StereoResult ret;

    m_resultMutex.lock();
    ret = m_result;
    m_resultMutex.unlock();

    return ret;
}

void ProcessorThread::run()
{
    if ( !m_frame.empty() ) {
        auto result = m_processor->process( m_frame );
        m_resultMutex.lock();
        m_result = result;
        m_resultMutex.unlock();

        emit frameProcessed();

    }

}
