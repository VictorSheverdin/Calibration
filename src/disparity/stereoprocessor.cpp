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
    m_matcher = cv::StereoBM::create();
}

int BMDisparityProcessor::getMinDisparity() const
{
    return m_matcher->getMinDisparity();
}

void BMDisparityProcessor::setMinDisparity( const int minDisparity )
{
    m_matcher->setMinDisparity( minDisparity );
}

int BMDisparityProcessor::getNumDisparities() const
{
    return m_matcher->getNumDisparities();
}

void BMDisparityProcessor::setNumDisparities( const int numDisparities )
{
    m_matcher->setNumDisparities( numDisparities );
}

int BMDisparityProcessor::getBlockSize() const
{
    return m_matcher->getBlockSize();
}

void BMDisparityProcessor::setBlockSize( const int blockSize )
{
    m_matcher->setBlockSize( blockSize );
}

int BMDisparityProcessor::getSpeckleWindowSize() const
{
    return m_matcher->getSpeckleWindowSize();
}

void BMDisparityProcessor::setSpeckleWindowSize( const int speckleWindowSize )
{
    m_matcher->setSpeckleWindowSize( speckleWindowSize );
}

int BMDisparityProcessor::getSpeckleRange() const
{
    return m_matcher->getSpeckleRange();
}

void BMDisparityProcessor::setSpeckleRange( const int speckleRange )
{
    m_matcher->setSpeckleRange( speckleRange );
}

int BMDisparityProcessor::getDisp12MaxDiff() const
{
    return m_matcher->getDisp12MaxDiff();
}

void BMDisparityProcessor::setDisp12MaxDiff( const int disp12MaxDiff )
{
    m_matcher->setDisp12MaxDiff( disp12MaxDiff );
}

int BMDisparityProcessor::getPreFilterType() const
{
    return m_matcher->getPreFilterType();
}

void BMDisparityProcessor::setPreFilterType( const int preFilterType )
{
    m_matcher->setPreFilterType( preFilterType );
}

int BMDisparityProcessor::getPreFilterSize() const
{
    return m_matcher->getPreFilterSize();
}

void BMDisparityProcessor::setPreFilterSize( const int preFilterSize )
{
    m_matcher->setPreFilterSize( preFilterSize );
}

int BMDisparityProcessor::getPreFilterCap() const
{
    return m_matcher->getPreFilterCap();
}

void BMDisparityProcessor::setPreFilterCap( const int preFilterCap )
{
    m_matcher->setPreFilterCap( preFilterCap );
}

int BMDisparityProcessor::getTextureThreshold() const
{
    return m_matcher->getTextureThreshold();
}

void BMDisparityProcessor::setTextureThreshold( const int textureThreshold )
{
    m_matcher->setTextureThreshold( textureThreshold );
}

int BMDisparityProcessor::getUniquenessRatio() const
{
    return m_matcher->getUniquenessRatio();
}

void BMDisparityProcessor::setUniquenessRatio( const int uniquenessRatio )
{
    m_matcher->setUniquenessRatio( uniquenessRatio );
}

cv::Rect BMDisparityProcessor::getROI1() const
{
    return m_matcher->getROI1();
}

void BMDisparityProcessor::setROI1( const cv::Rect &roi1 )
{
    m_matcher->setROI1( roi1 );
}

cv::Rect BMDisparityProcessor::getROI2() const
{
    return m_matcher->getROI2();
}

void BMDisparityProcessor::setROI2( const cv::Rect &roi2 )
{
    m_matcher->setROI2( roi2 );
}

cv::Mat BMDisparityProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    CvImage leftGray;
    CvImage rightGray;

    cv::cvtColor( left,  leftGray,  cv::COLOR_BGR2GRAY );
    cv::cvtColor( right, rightGray, cv::COLOR_BGR2GRAY );

    cv::Mat leftDisp;
    cv::Mat rightDisp;

    m_matcher->compute( leftGray, rightGray, leftDisp );

/*    cv::Ptr< cv::ximgproc::DisparityWLSFilter > wlsFilter;
    wlsFilter = cv::ximgproc::createDisparityWLSFilter( m_matcher );

    auto rightMatcher = cv::ximgproc::createRightMatcher(m_matcher);
    rightMatcher->compute( rightGray, leftGray, rightDisp );

    wlsFilter->setLambda( 8000.0 );
    wlsFilter->setSigmaColor( 1.5 );

    cv::Mat filteredDisp;

    wlsFilter->filter( leftDisp, left, filteredDisp, rightDisp );*/

    return leftDisp;

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
    cv::Mat rightDisp;

    m_matcher->compute( leftGray, rightGray, leftDisp );

/*    auto rightMatcher = cv::ximgproc::createRightMatcher(m_matcher);
    rightMatcher->compute( rightGray, leftGray, rightDisp );*/

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

const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &StereoResult::pointCloud() const
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

void StereoProcessor::setCalibration( const StereoCalibrationData &data )
{
    m_calibration = data;
}

const StereoCalibrationData &StereoProcessor::calibration() const
{
    return m_calibration;
}

bool StereoProcessor::loadYaml( const std::string &fileName )
{
    m_calibration.loadYaml( fileName );
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


            auto previewImage = stackImages( leftRectifiedImage, rightRectifiedImage );
            drawTraceLines( previewImage, 20 );

            ret.setPreviewImage( previewImage );

            if ( m_disparityProcessor ) {

                auto disparity = m_disparityProcessor->processDisparity( leftRectifiedImage, rightRectifiedImage );

                ret.setDisparity( disparity );

                cv::Mat points;

                cv::reprojectImageTo3D( disparity, points, m_calibration.disparityToDepthMatrix(), true );

                ret.setPoints( points );

                pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointCloud = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );

                pointCloud->clear();

                for (int rows = 0; rows < points.rows; ++rows) {
                    for (int cols = 0; cols < points.cols; ++cols) {

                        cv::Point3f point = points.at< cv::Point3f >(rows, cols);

                        if ( point.x > -10 && point.y > -10 && point.z > -10 &&
                             point.x < 10 && point.y < 10 && point.z < 10) {

                            pcl::PointXYZRGB pclPoint;
                            pclPoint.x = point.x;
                            pclPoint.y = point.y;
                            pclPoint.z = point.z;

                            cv::Vec3b intensity = leftRectifiedImage.at<cv::Vec3b>(rows,cols); //BGR
                            uint32_t rgb = (static_cast<uint32_t>(intensity[2]) << 16 | static_cast<uint32_t>(intensity[1]) << 8 | static_cast<uint32_t>(intensity[0]));
                            pclPoint.rgb = *reinterpret_cast<float*>(&rgb);
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

