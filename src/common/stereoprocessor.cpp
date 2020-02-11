#include "src/common/precompiled.h"

#include "stereoprocessor.h"

#include "src/common/functions.h"

const double MISSING_Z = 10000.;

// DisparityProcessorBase
DisparityProcessorBase::DisparityProcessorBase()
{
}

CvImage DisparityProcessorBase::preprocess( const CvImage img )
{
    CvImage ret;

    cv::cvtColor( img,  ret,  cv::COLOR_BGR2GRAY );

    cv::equalizeHist( ret, ret );
    cv::GaussianBlur( ret, ret, cv::Size (5,5), 0);

    return ret;

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
    CvImage leftGray = preprocess( left );
    CvImage rightGray = preprocess( right );

    cv::Mat leftDisp;

    m_leftMatcher->compute( leftGray, rightGray, leftDisp );

    // std::cout << leftDisp;

    // cv::normalize( leftDisp, leftDisp, 0.0, 255, cv::NORM_MINMAX, CV_8U );

    return leftDisp;

//    cv::Mat rightDisp;

//    auto rightMatcher = cv::ximgproc::createRightMatcher( m_leftMatcher );

//    rightMatcher->compute( rightGray, leftGray, rightDisp );

//    auto wlsFilter = cv::ximgproc::createDisparityWLSFilter( m_leftMatcher );

//    wlsFilter->setLambda( 8000.0 );
//    wlsFilter->setSigmaColor( 1.5 );

//    cv::Mat conf_map = cv::Mat( left.rows, left.cols, CV_8U );
//    conf_map = cv::Scalar(255);

//    cv::Mat filteredDisp;

//    wlsFilter->filter( leftDisp, left, filteredDisp, rightDisp );

//    conf_map = wlsFilter->getConfidenceMap();

//    cv::Mat solvedDisp;
//    cv::Mat solvedFilteredDisp;

//    cv::ximgproc::fastBilateralSolverFilter( left, leftDisp, conf_map/255.0f, solvedDisp );
//    cv::ximgproc::fastBilateralSolverFilter( left, filteredDisp, conf_map/255.0f, solvedFilteredDisp );

//    return solvedFilteredDisp;

}

// BMGPUDisparityProcessor
BMGPUDisparityProcessor::BMGPUDisparityProcessor()
    : DisparityProcessorBase()
{
    initialize();
}

void BMGPUDisparityProcessor::initialize()
{
    m_matcher = cv::cuda::createStereoBM();
}

int BMGPUDisparityProcessor::getNumDisparities() const
{
    return m_matcher->getNumDisparities();
}

void BMGPUDisparityProcessor::setNumDisparities( const int numDisparities )
{
    m_matcher->setNumDisparities( numDisparities );
}

int BMGPUDisparityProcessor::getBlockSize() const
{
    return m_matcher->getBlockSize();
}

void BMGPUDisparityProcessor::setBlockSize( const int blockSize )
{
    m_matcher->setBlockSize( blockSize );
}

int BMGPUDisparityProcessor::getPreFilterType() const
{
    return m_matcher->getPreFilterType();
}

void BMGPUDisparityProcessor::setPreFilterType( const int preFilterType )
{
    m_matcher->setPreFilterType( preFilterType );
}

int BMGPUDisparityProcessor::getPreFilterCap() const
{
    return m_matcher->getPreFilterCap();
}

void BMGPUDisparityProcessor::setPreFilterCap( const int preFilterCap )
{
    m_matcher->setPreFilterCap( preFilterCap );
}

int BMGPUDisparityProcessor::getTextureThreshold() const
{
    return m_matcher->getTextureThreshold();
}

void BMGPUDisparityProcessor::setTextureThreshold( const int textureThreshold )
{
    m_matcher->setTextureThreshold( textureThreshold );
}

cv::Mat BMGPUDisparityProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    cv::Mat leftGray = preprocess( left );
    cv::Mat rightGray = preprocess( right );

    cv::cuda::GpuMat leftGPU;
    cv::cuda::GpuMat rightGPU;

    leftGPU.upload( leftGray );
    rightGPU.upload( rightGray );

    cv::cuda::GpuMat dispGPU( leftGray.size(), CV_8U );

    m_matcher->compute( leftGPU, rightGPU, dispGPU );

    cv::Ptr < cv::cuda::DisparityBilateralFilter > dispFilter = cv::cuda::createDisparityBilateralFilter( m_matcher->getNumDisparities(), 5, 1 );
    dispFilter->apply( dispGPU, leftGPU, dispGPU );

    cv::cuda::normalize( dispGPU, dispGPU, 0, 255, cv::NORM_MINMAX, CV_8U );
    cv::Mat res( leftGray.size(), CV_8U );

    dispGPU.download( res );

    cv::Mat floatRes;

    res.convertTo( floatRes, CV_32F );

    // std::cout << floatRes;

    return floatRes;

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
    CvImage leftGray = preprocess( left );
    CvImage rightGray = preprocess( right );

    cv::Mat leftDisp;

    m_matcher->compute( leftGray, rightGray, leftDisp );

    return leftDisp;

}

// BPDisparityProcessor
BPDisparityProcessor::BPDisparityProcessor()
    : DisparityProcessorBase()
{
    initialize();
}

void BPDisparityProcessor::initialize()
{
    m_matcher = cv::cuda::createStereoBeliefPropagation();

    setMsgType( CV_16S );
}

int BPDisparityProcessor::getNumDisparities() const
{
    return m_matcher->getNumDisparities();
}

void BPDisparityProcessor::setNumDisparities( const int value )
{
    m_matcher->setNumDisparities( value );
}

int BPDisparityProcessor::getNumIterations() const
{
    return m_matcher->getNumIters();
}

void BPDisparityProcessor::setNumIterations( const int value )
{
    m_matcher->setNumIters( value );
}

int BPDisparityProcessor::getNumLevels() const
{
    return m_matcher->getNumLevels();
}

void BPDisparityProcessor::setNumLevels( const int value )
{
    m_matcher->setNumLevels( value );
}

double BPDisparityProcessor::getMaxDataTerm() const
{
    return m_matcher->getMaxDataTerm();
}

void BPDisparityProcessor::setMaxDataTerm( const double value )
{
    m_matcher->setMaxDataTerm( value );
}

double BPDisparityProcessor::getDataWeight() const
{
    return m_matcher->getDataWeight();
}

void BPDisparityProcessor::setDataWeight( const double value )
{
    m_matcher->setDataWeight( value );
}

double BPDisparityProcessor::getMaxDiscTerm() const
{
    return m_matcher->getMaxDiscTerm();
}

void BPDisparityProcessor::setMaxDiscTerm( const double value )
{
    m_matcher->setMaxDiscTerm( value );
}

double BPDisparityProcessor::getDiscSingleJump() const
{
    return m_matcher->getDiscSingleJump();
}

void BPDisparityProcessor::setDiscSingleJump( const double value )
{
    m_matcher->setDiscSingleJump( value );
}

int BPDisparityProcessor::getMsgType() const
{
    return m_matcher->getMsgType();
}

void BPDisparityProcessor::setMsgType( const int value )
{
    m_matcher->setMsgType( value );
}

cv::Mat BPDisparityProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    CvImage leftGray;
    CvImage rightGray;

    leftGray = preprocess( left );
    rightGray = preprocess( right );

    cv::resize( leftGray, leftGray, cv::Size(), 0.5, 0.5 );
    cv::resize( rightGray, rightGray, cv::Size(), 0.5, 0.5 );

    cv::cuda::GpuMat leftGPU;
    cv::cuda::GpuMat rightGPU;

    leftGPU.upload( leftGray );
    rightGPU.upload( rightGray );

    cv::cuda::GpuMat leftDisp;

    m_matcher->compute( leftGPU, rightGPU, leftDisp );

    // cv::cuda::normalize( leftDisp, leftDisp, 0, 255, cv::NORM_MINMAX, CV_8U );

    cv::Mat res;

    leftDisp.download( res );

    return res;

}

// CSBPDisparityProcessor
CSBPDisparityProcessor::CSBPDisparityProcessor()
    : DisparityProcessorBase()
{
    initialize();
}

void CSBPDisparityProcessor::initialize()
{
    m_matcher = cv::cuda::createStereoConstantSpaceBP();
}

cv::Mat CSBPDisparityProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    CvImage leftGray;
    CvImage rightGray;

    leftGray = preprocess( left );
    rightGray = preprocess( right );

    cv::cuda::GpuMat leftGPU;
    cv::cuda::GpuMat rightGPU;

    leftGPU.upload( leftGray );
    rightGPU.upload( rightGray );

    cv::cuda::GpuMat leftDisp;

    m_matcher->compute( leftGPU, rightGPU, leftDisp );

    // cv::cuda::normalize( leftDisp, leftDisp, 0, 255, cv::NORM_MINMAX, CV_8U );

    cv::Mat res;

    leftDisp.download( res );

    return res;

}

// StereoProcessorBase
StereoProcessorBase::StereoProcessorBase()
{
}

StereoProcessorBase::StereoProcessorBase( const std::shared_ptr< DisparityProcessorBase > &proc )
{
    setDisparityProcessor( proc );
}

const std::shared_ptr< DisparityProcessorBase > &StereoProcessorBase::disparityProcessor() const
{
    return m_disparityProcessor;
}

void StereoProcessorBase::setDisparityProcessor( const std::shared_ptr< DisparityProcessorBase > &proc )
{
    m_disparityProcessor = proc;
}

cv::Mat StereoProcessorBase::processDisparity( const CvImage &left, const CvImage &right )
{
    if ( !m_disparityProcessor )
        return cv::Mat();

    return m_disparityProcessor->processDisparity( left, right );
}

// StereoProcessor
StereoProcessor::StereoProcessor()
{
}

StereoProcessor::StereoProcessor( const std::shared_ptr< DisparityProcessorBase > &proc )
    : StereoProcessorBase( proc )
{
}

void StereoProcessor::setDisparityToDepthMatrix( const cv::Mat &mat )
{
    m_disparityToDepthMatrix = mat;
}

const cv::Mat &StereoProcessor::disparityToDepthMatrix() const
{
    return m_disparityToDepthMatrix;
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr StereoProcessor::processPointCloud( const CvImage &left, const CvImage &right )
{
    auto disparity = processDisparity( left, right );

    if ( !disparity.empty() ) {
        auto points = reprojectPoints( disparity );

        if ( !points.empty() )
            return producePointCloud( points, left );

    }

    return pcl::PointCloud< pcl::PointXYZRGB >::Ptr();

}

std::list< ColorPoint3d > StereoProcessor::processPointList( const CvImage &left, const CvImage &right )
{
    auto disparity = processDisparity( left, right );

    if ( !disparity.empty() ) {
        auto points = reprojectPoints( disparity );

        if ( !points.empty() )
            return producePointList( points, left );

    }

    return std::list< ColorPoint3d >();

}

cv::Mat StereoProcessor::reprojectPoints( const cv::Mat &disparity )
{
    cv::Mat points;

    cv::reprojectImageTo3D( disparity, points, m_disparityToDepthMatrix, true, CV_32F );

    return points;
}

bool isValidPoint( const cv::Vec3f& pt )
{
    return pt[2] != MISSING_Z && !std::isinf( pt[2] );
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr StereoProcessor::producePointCloud( const cv::Mat &points, const CvImage &leftImage )
{
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointCloud = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );

    CvImage rgbLeftImage;

    cv::cvtColor( leftImage, rgbLeftImage, cv::COLOR_BGR2RGB );

    for (int rows = 0; rows < points.rows; ++rows) {

        for (int cols = 0; cols < points.cols; ++cols) {

            cv::Point3f point = points.at< cv::Point3f >(rows, cols);

            if ( isValidPoint( point ) ) {

                pcl::PointXYZRGB pclPoint;
                pclPoint.x = point.x;
                pclPoint.y = point.y;
                pclPoint.z = point.z;

                cv::Vec3b intensity = rgbLeftImage.at< cv::Vec3b >( rows, cols );
                pclPoint.r = intensity[0];
                pclPoint.g = intensity[1];
                pclPoint.b = intensity[2];
                pointCloud->push_back( pclPoint );

            }

        }

    }

    return pointCloud;

}

std::list< ColorPoint3d > StereoProcessor::producePointList( const cv::Mat &points, const CvImage &leftImage )
{
    std::list< ColorPoint3d > ret;

    CvImage rgbLeftImage;

    cv::cvtColor( leftImage, rgbLeftImage, cv::COLOR_BGR2RGB );

    for (int rows = 0; rows < points.rows; ++rows) {

        for (int cols = 0; cols < points.cols; ++cols) {

            cv::Point3f point = points.at< cv::Point3f >( rows, cols );

            if ( isValidPoint( point ) ) {

                cv::Vec3b intensity = rgbLeftImage.at< cv::Vec3b >( rows, cols );
                cv::Scalar color( intensity[ 2 ], intensity[ 1 ], intensity[ 0 ], 255 );
                ColorPoint3d colorPoint( point * 15, color ) ;

                ret.push_back( colorPoint );

            }

        }

    }

    return ret;
}

// BMStereoProcessor
BMStereoProcessor::BMStereoProcessor()
{
    initialize();
}

void BMStereoProcessor::initialize()
{
    setDisparityProcessor( std::make_shared< BMDisparityProcessor >() );
}

const std::shared_ptr< BMDisparityProcessor > BMStereoProcessor::disparityProcessor() const
{
    return std::dynamic_pointer_cast< BMDisparityProcessor >( m_disparityProcessor );
}

int BMStereoProcessor::getMinDisparity() const
{
    return disparityProcessor()->getMinDisparity();
}

void BMStereoProcessor::setMinDisparity( const int minDisparity )
{
    disparityProcessor()->setMinDisparity( minDisparity );
}

int BMStereoProcessor::getNumDisparities() const
{
    return disparityProcessor()->getNumDisparities();
}

void BMStereoProcessor::setNumDisparities( const int numDisparities )
{
    disparityProcessor()->setNumDisparities( numDisparities );
}

int BMStereoProcessor::getBlockSize() const
{
    return disparityProcessor()->getBlockSize();
}

void BMStereoProcessor::setBlockSize( const int blockSize )
{
    disparityProcessor()->setBlockSize( blockSize );
}

int BMStereoProcessor::getTextureThreshold() const
{
    return disparityProcessor()->getTextureThreshold();
}

void BMStereoProcessor::setTextureThreshold( const int textureThreshold )
{
    disparityProcessor()->setTextureThreshold( textureThreshold );
}

int BMStereoProcessor::getSpeckleWindowSize() const
{
    return disparityProcessor()->getSpeckleWindowSize();
}

void BMStereoProcessor::setSpeckleWindowSize( const int speckleWindowSize )
{
    disparityProcessor()->setSpeckleWindowSize( speckleWindowSize );
}

int BMStereoProcessor::getSpeckleRange() const
{
    return disparityProcessor()->getSpeckleRange();
}

void BMStereoProcessor::setSpeckleRange( const int speckleRange )
{
    disparityProcessor()->setSpeckleRange( speckleRange );
}

int BMStereoProcessor::getDisp12MaxDiff() const
{
    return disparityProcessor()->getDisp12MaxDiff();
}

void BMStereoProcessor::setDisp12MaxDiff( const int disp12MaxDiff )
{
    disparityProcessor()->setDisp12MaxDiff( disp12MaxDiff );
}

int BMStereoProcessor::getPreFilterSize() const
{
    return disparityProcessor()->getPreFilterSize();
}

void BMStereoProcessor::setPreFilterSize( const int preFilterSize )
{
    disparityProcessor()->setPreFilterSize( preFilterSize );
}

int BMStereoProcessor::getPreFilterCap() const
{
    return disparityProcessor()->getPreFilterCap();
}

void BMStereoProcessor::setPreFilterCap( const int preFilterCap )
{
    disparityProcessor()->setPreFilterCap( preFilterCap );
}

int BMStereoProcessor::getUniquenessRatio() const
{
    return disparityProcessor()->getUniquenessRatio();
}

void BMStereoProcessor::setUniquenessRatio( const int uniquenessRatio )
{
    disparityProcessor()->setUniquenessRatio( uniquenessRatio );
}
