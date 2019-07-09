#include "src/common/precompiled.h"

#include "stereoprocessor.h"

#include "src/common/functions.h"

// DisparityProcessorBase
DisparityProcessorBase::DisparityProcessorBase()
{
}

// BMDisparityProcessor
BMDisparityProcessor::BMDisparityProcessor()
    : DisparityProcessorBase()
{
    initialize();
}

void BMDisparityProcessor::initialize()
{
    m_leftMatcher = cv::StereoBM::create();

}

int BMDisparityProcessor::getMinDisparity() const
{
    return m_leftMatcher->getMinDisparity();
}

void BMDisparityProcessor::setMinDisparity( const int minDisparity )
{
    m_leftMatcher->setMinDisparity( minDisparity );
}

int BMDisparityProcessor::getNumDisparities() const
{
    return m_leftMatcher->getNumDisparities();
}

void BMDisparityProcessor::setNumDisparities( const int numDisparities )
{
    m_leftMatcher->setNumDisparities( numDisparities );
}

int BMDisparityProcessor::getBlockSize() const
{
    return m_leftMatcher->getBlockSize();
}

void BMDisparityProcessor::setBlockSize( const int blockSize )
{
    m_leftMatcher->setBlockSize( blockSize );
}

int BMDisparityProcessor::getSpeckleWindowSize() const
{
    return m_leftMatcher->getSpeckleWindowSize();
}

void BMDisparityProcessor::setSpeckleWindowSize( const int speckleWindowSize )
{
    m_leftMatcher->setSpeckleWindowSize( speckleWindowSize );
}

int BMDisparityProcessor::getSpeckleRange() const
{
    return m_leftMatcher->getSpeckleRange();
}

void BMDisparityProcessor::setSpeckleRange( const int speckleRange )
{
    m_leftMatcher->setSpeckleRange( speckleRange );
}

int BMDisparityProcessor::getDisp12MaxDiff() const
{
    return m_leftMatcher->getDisp12MaxDiff();
}

void BMDisparityProcessor::setDisp12MaxDiff( const int disp12MaxDiff )
{
    m_leftMatcher->setDisp12MaxDiff( disp12MaxDiff );
}

int BMDisparityProcessor::getPreFilterType() const
{
    return m_leftMatcher->getPreFilterType();
}

void BMDisparityProcessor::setPreFilterType( const int preFilterType )
{
    m_leftMatcher->setPreFilterType( preFilterType );
}

int BMDisparityProcessor::getPreFilterSize() const
{
    return m_leftMatcher->getPreFilterSize();
}

void BMDisparityProcessor::setPreFilterSize( const int preFilterSize )
{
    m_leftMatcher->setPreFilterSize( preFilterSize );
}

int BMDisparityProcessor::getPreFilterCap() const
{
    return m_leftMatcher->getPreFilterCap();
}

void BMDisparityProcessor::setPreFilterCap( const int preFilterCap )
{
    m_leftMatcher->setPreFilterCap( preFilterCap );
}

int BMDisparityProcessor::getTextureThreshold() const
{
    return m_leftMatcher->getTextureThreshold();
}

void BMDisparityProcessor::setTextureThreshold( const int textureThreshold )
{
    m_leftMatcher->setTextureThreshold( textureThreshold );
}

int BMDisparityProcessor::getUniquenessRatio() const
{
    return m_leftMatcher->getUniquenessRatio();
}

void BMDisparityProcessor::setUniquenessRatio( const int uniquenessRatio )
{
    m_leftMatcher->setUniquenessRatio( uniquenessRatio );
}

cv::Rect BMDisparityProcessor::getROI1() const
{
    return m_leftMatcher->getROI1();
}

void BMDisparityProcessor::setROI1( const cv::Rect &roi1 )
{
    m_leftMatcher->setROI1( roi1 );
}

cv::Rect BMDisparityProcessor::getROI2() const
{
    return m_leftMatcher->getROI2();
}

void BMDisparityProcessor::setROI2( const cv::Rect &roi2 )
{
    m_leftMatcher->setROI2( roi2 );
}

cv::Mat BMDisparityProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    CvImage leftGray;
    CvImage rightGray;

    cv::cvtColor( left,  leftGray,  cv::COLOR_BGR2GRAY );
    cv::cvtColor( right, rightGray, cv::COLOR_BGR2GRAY );

    cv::Mat leftDisp;

    m_leftMatcher->compute( leftGray, rightGray, leftDisp );

    return leftDisp;
/*
    cv::Mat rightDisp;

    auto rightMatcher = cv::ximgproc::createRightMatcher( m_leftMatcher );

    rightMatcher->compute( rightGray, leftGray, rightDisp );

    auto wlsFilter = cv::ximgproc::createDisparityWLSFilter( m_leftMatcher );

    wlsFilter->setLambda( 8000.0 );
    wlsFilter->setSigmaColor( 1.5 );

    cv::Mat conf_map = cv::Mat( left.rows, left.cols, CV_8U );
    conf_map = cv::Scalar(255);

    cv::Mat filteredDisp;

    wlsFilter->filter( leftDisp, left, filteredDisp, rightDisp );

    conf_map = wlsFilter->getConfidenceMap();

    cv::Mat solvedDisp;
    cv::Mat solvedFilteredDisp;

    cv::ximgproc::fastBilateralSolverFilter( left, leftDisp, conf_map/255.0f, solvedDisp );
    cv::ximgproc::fastBilateralSolverFilter( left, filteredDisp, conf_map/255.0f, solvedFilteredDisp );

    return solvedFilteredDisp;*/

}

// GMDisparityProcessor
GMDisparityProcessor::GMDisparityProcessor()
    : DisparityProcessorBase()
{
    initialize();
}

void GMDisparityProcessor::initialize()
{
    m_matcher = cv::StereoSGBM::create();
}

int GMDisparityProcessor::getMode() const
{
    return m_matcher->getMode();
}

void GMDisparityProcessor::setMode( int mode )
{
    m_matcher->setMode( mode );
}

int GMDisparityProcessor::getMinDisparity() const
{
    return m_matcher->getMinDisparity();
}

void GMDisparityProcessor::setMinDisparity( const int minDisparity )
{
    m_matcher->setMinDisparity( minDisparity );
}

int GMDisparityProcessor::getNumDisparities() const
{
    return m_matcher->getNumDisparities();
}

void GMDisparityProcessor::setNumDisparities( const int numDisparities )
{
    m_matcher->setNumDisparities( numDisparities );
}

int GMDisparityProcessor::getBlockSize() const
{
    return m_matcher->getBlockSize();
}

void GMDisparityProcessor::setBlockSize( const int blockSize )
{
    m_matcher->setBlockSize( blockSize );
}

int GMDisparityProcessor::getSpeckleWindowSize() const
{
    return m_matcher->getSpeckleWindowSize();
}

void GMDisparityProcessor::setSpeckleWindowSize( const int speckleWindowSize )
{
    m_matcher->setSpeckleWindowSize( speckleWindowSize );
}

int GMDisparityProcessor::getSpeckleRange() const
{
    return m_matcher->getSpeckleRange();
}

void GMDisparityProcessor::setSpeckleRange( const int speckleRange )
{
    m_matcher->setSpeckleRange( speckleRange );
}

int GMDisparityProcessor::getDisp12MaxDiff() const
{
    return m_matcher->getDisp12MaxDiff();
}

void GMDisparityProcessor::setDisp12MaxDiff( const int disp12MaxDiff )
{
    m_matcher->setDisp12MaxDiff( disp12MaxDiff );
}

int GMDisparityProcessor::getPreFilterCap() const
{
    return m_matcher->getPreFilterCap();
}

void GMDisparityProcessor::setPreFilterCap( const int preFilterCap )
{
    m_matcher->setPreFilterCap( preFilterCap );
}

int GMDisparityProcessor::getUniquenessRatio() const
{
    return m_matcher->getUniquenessRatio();
}

void GMDisparityProcessor::setUniquenessRatio( const int uniquenessRatio )
{
    m_matcher->setUniquenessRatio( uniquenessRatio );
}

int GMDisparityProcessor::getP1() const
{
    return m_matcher->getP1();
}

void GMDisparityProcessor::setP1(int p1)
{
    m_matcher->setP1( p1 );
}

int GMDisparityProcessor::getP2() const
{
    return m_matcher->getP2();
}

void GMDisparityProcessor::setP2(int p2)
{
    m_matcher->setP2( p2 );
}

cv::Mat GMDisparityProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    CvImage leftGray;
    CvImage rightGray;

    cv::cvtColor( left,  leftGray,  cv::COLOR_BGR2GRAY );
    cv::cvtColor( right, rightGray, cv::COLOR_BGR2GRAY );

    cv::Mat leftDisp;

    m_matcher->compute( leftGray, rightGray, leftDisp );

    return leftDisp;

}

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

// StereoProcessor
StereoProcessor::StereoProcessor()
{
    initialize();
}

void StereoProcessor::initialize()
{
}

void StereoProcessor::setCalibration( const StereoCalibrationDataShort &data )
{
    m_calibration = data;
}

const StereoCalibrationDataShort &StereoProcessor::calibration() const
{
    return m_calibration;
}

bool StereoProcessor::loadYaml( const std::string &fileName )
{
    return m_calibration.loadYaml( fileName );
}

std::shared_ptr< DisparityProcessorBase > StereoProcessor::disparityProcessor() const
{
    return m_disparityProcessor;
}

void StereoProcessor::setDisparityProcessor(const std::shared_ptr< DisparityProcessorBase > proc )
{
    m_disparityProcessor = proc;
}

StereoResult StereoProcessor::process( const CvImage &leftFrame, const CvImage &rightFrame )
{
    StereoResult ret;

    if ( !leftFrame.empty() && !rightFrame.empty() ) {

        if ( m_calibration.isOk() ) {

            CvImage leftRectifiedImage;
            CvImage rightRectifiedImage;

            cv::remap( leftFrame, leftRectifiedImage, m_calibration.leftRMap(), m_calibration.leftDMap(), cv::INTER_LINEAR );
            cv::remap( rightFrame, rightRectifiedImage, m_calibration.rightRMap(), m_calibration.rightDMap(), cv::INTER_LINEAR );

            CvImage leftCroppedFrame;
            CvImage rightCroppedFrame;

            if (!m_calibration.leftROI().empty() && !m_calibration.rightROI().empty()) {
                leftCroppedFrame = leftRectifiedImage( m_calibration.leftROI() );
                rightCroppedFrame = rightRectifiedImage( m_calibration.leftROI() );
            }

            auto previewImage = stackImages( leftCroppedFrame, rightCroppedFrame );
            drawTraceLines( previewImage, 20 );

            ret.setPreviewImage( previewImage );

            if ( m_disparityProcessor ) {

                auto disparity = m_disparityProcessor->processDisparity( leftCroppedFrame, rightCroppedFrame );

                ret.setDisparity( disparity );

                cv::Mat points;

                cv::reprojectImageTo3D( disparity, points, m_calibration.disparityToDepthMatrix(), true );

                ret.setPoints( points );

                pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointCloud = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );

                pointCloud->clear();

                CvImage rgbLeftImage;

                cv::cvtColor( leftCroppedFrame, rgbLeftImage, cv::COLOR_BGR2RGB );

                for (int rows = 0; rows < points.rows; ++rows) {                    
                    for (int cols = 0; cols < points.cols; ++cols) {

                        cv::Point3f point = points.at< cv::Point3f >(rows, cols);

                        if ( point.x > -1000 && point.y > -1000 && point.z > 0 &&
                             point.x < 1000 && point.y < 1000 && point.z < 1000 ) {

                            pcl::PointXYZRGB pclPoint;
                            pclPoint.x = -point.x;
                            pclPoint.y = -point.y;
                            pclPoint.z = point.z;

                            cv::Vec3b intensity = rgbLeftImage.at< cv::Vec3b >( rows, cols );
                            uint32_t rgb = (static_cast<uint32_t>(intensity[0]) << 16 | static_cast<uint32_t>(intensity[1]) << 8 | static_cast<uint32_t>(intensity[2]));
                            pclPoint.rgb = *reinterpret_cast< float* >( &rgb );
                            pointCloud->push_back( pclPoint );

                        }

                    }

                }

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

void ProcessorThread::queueImage( const StereoImage &img )
{
    m_processMutex.lock();

    if ( m_processQueue.size() >= m_maxDequeSize )
        m_processQueue.resize( m_maxDequeSize - 1 );

    m_processQueue.push_front( img );

    m_processMutex.unlock();

}
void ProcessorThread::run()
{
}

