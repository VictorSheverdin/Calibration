#include "precompiled.h"

#include "featureprocessor.h"

#include "defs.h"

void extractKeypoints( cv::Ptr< cv::Feature2D > processor, const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints )
{
    if ( processor && keypoints ) {

        auto uImage = image.getUMat( cv::ACCESS_READ );
        processor->detect( uImage, *keypoints, mask );

    }

}

void extractDescriptors( cv::Ptr< cv::Feature2D > processor, const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors )
{
    if ( processor && descriptors ) {
        processor->compute( image, keypoints, *descriptors );

    }

}

void extractAndCompute( cv::Ptr< cv::Feature2D > processor, const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors )
{
    if ( processor && keypoints && descriptors ) {
        processor->detectAndCompute( image, mask, *keypoints, *descriptors );

    }

}

// TrackPointResult
FlowTrackResult::FlowTrackResult( size_t index, cv::Point2f point, float error , float miss )
    : cv::Point2f( point ), index( index ), error( error ), miss( miss )
{
}

bool FlowTrackResult::operator<( const FlowTrackResult &other ) const
{
    return index < other.index;
}

// FlowProcessor
const double FlowProcessor::m_checkDistance = 1.0;

// FlowProcessor
FlowProcessor::FlowProcessor()
{
    initialize();
}

void FlowProcessor::initialize()
{
    m_extractPrecision = 1.e-2;
    m_blockSize = 5;
    m_ransacReprojectionThreshold = 1.0;
    m_ransacConfidence = 1.0 - 1.e-3;
}

void FlowProcessor::extractPoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::Point2f > *points , const size_t count, const double distance)
{
    if ( points ) {

        cv::Mat gray;
        cv::cvtColor( image, gray, cv::COLOR_BGR2GRAY );

        cv::goodFeaturesToTrack( gray, *points, count, m_extractPrecision, distance, mask, m_blockSize );

    }

}

double FlowProcessor::extractPrecision() const
{
    return m_extractPrecision;
}

void FlowProcessor::setExtractPrecision( const double value )
{
    m_extractPrecision = value;
}

double FlowProcessor::ransacReprojectionThreshold() const
{
    return m_ransacReprojectionThreshold;
}

void FlowProcessor::setRansacReprojectionThreshold( const double &value )
{
    m_ransacReprojectionThreshold = value;
}

double FlowProcessor::ransacConfidence() const
{
    return m_ransacConfidence;
}

void FlowProcessor::setRansacConfidence( const double &value )
{
    m_ransacConfidence = value;
}

cv::Mat FlowProcessor::epiTest( const std::vector< cv::Point2f > &sourcePoints, const std::vector< cv::Point2f > &targetPoints, std::vector< int > *inliers )
{
    if ( inliers ) {

        if ( sourcePoints.size() == targetPoints.size() && sourcePoints.size() > MIN_FMAT_POINTS_COUNT ) {

            std::vector< uchar > inlierFlags( sourcePoints.size(), 0 );

            auto fmat = cv::findFundamentalMat( sourcePoints, targetPoints, inlierFlags, cv::FM_RANSAC, m_ransacReprojectionThreshold, m_ransacConfidence );

            inliers->clear();
            inliers->reserve( inlierFlags.size() );

            for ( size_t i = 0; i < inlierFlags.size(); ++i )
                if ( inlierFlags[ i ] )
                    inliers->push_back( i );

            return fmat;

        }

    }

    return cv::Mat();

}

// GPUFlowProcessor
GPUFlowProcessor::GPUFlowProcessor()
{
    initialize();
}

void GPUFlowProcessor::initialize()
{
    m_opticalProcessor = cv::cuda::SparsePyrLKOpticalFlow::create();
}

size_t GPUFlowProcessor::winSize() const
{
    auto size = m_opticalProcessor->getWinSize();

    return static_cast< size_t >( std::max( 0, std::min( size.width, size.height ) ) );
}

void GPUFlowProcessor::setWinSize( const size_t value )
{
    m_opticalProcessor->setWinSize( cv::Size( value, value ) );
}

size_t GPUFlowProcessor::levels() const
{
    return static_cast< size_t >( std::max( 0, m_opticalProcessor->getMaxLevel() ) );
}

void GPUFlowProcessor::setLevels( const size_t value )
{
    m_opticalProcessor->setMaxLevel( value );
}

void download( const cv::cuda::GpuMat &d_mat, std::vector< cv::Point2f > &vec )
{
    vec.resize( d_mat.cols );
    cv::Mat mat( 1, d_mat.cols, CV_32FC2, ( void* )&vec[ 0 ] );
    d_mat.download( mat );
}

void download( const cv::cuda::GpuMat &d_mat, std::vector< uchar > &vec )
{
    vec.resize( d_mat.cols );
    cv::Mat mat( 1, d_mat.cols, CV_8UC1, ( void* )&vec[ 0 ] );
    d_mat.download( mat );
}

void download( const cv::cuda::GpuMat &d_mat, std::vector< float > &vec )
{
    vec.resize( d_mat.cols );
    cv::Mat mat( 1, d_mat.cols, CV_32FC1, ( void* )&vec[ 0 ] );
    d_mat.download( mat );
}

void GPUFlowProcessor::track( const CvImage &sourceImage, const std::vector< cv::Point2f > &sourcePoints, const CvImage &targetImage, std::vector< FlowTrackResult > *trackedPoints )
{    
    if ( trackedPoints && !sourcePoints.empty() ) {

        std::vector< cv::Point2f > opticalPoints;
        std::vector< cv::Point2f > checkPoints;

        std::vector< unsigned char > statuses;
        std::vector< float > err;

        std::vector< unsigned char > checkStatuses;
        std::vector< float > checkErr;

        m_gpuSourceImage.upload( sourceImage );
        m_gpuTargetImage.upload( targetImage );
        m_gpuSourcePoints.upload( sourcePoints );

        m_opticalProcessor->calc( m_gpuSourceImage, m_gpuTargetImage, m_gpuSourcePoints, m_gpuOpticalPoints, m_gpuStatuses, m_gpuErr );
        m_opticalProcessor->calc( m_gpuTargetImage, m_gpuSourceImage, m_gpuOpticalPoints, m_gpuCheckPoints, m_gpuCheckStatuses, m_gpuCheckErr );

        download( m_gpuOpticalPoints, opticalPoints );
        download( m_gpuCheckPoints, checkPoints );
        download( m_gpuStatuses, statuses );
        download( m_gpuCheckStatuses, checkStatuses );
        download( m_gpuErr, err );
        download( m_gpuCheckErr, checkErr );

        trackedPoints->clear();

        for ( size_t i = 0; i < statuses.size(); ++i ) {

            auto miss = cv::norm( checkPoints[ i ] - sourcePoints[ i ] );

            if ( statuses[i] && checkStatuses[i] && miss < m_checkDistance
                 && opticalPoints[ i ].x >= 0 && opticalPoints[ i ].y >= 0
                 && opticalPoints[ i ].x < targetImage.cols &&  opticalPoints[ i ].y < targetImage.rows ) {

                trackedPoints->push_back( FlowTrackResult( i, opticalPoints[ i ], err[i], miss ) );

            }

        }

    }

}

// CPUFlowProcessor
CPUFlowProcessor::CPUFlowProcessor()
{
    initialize();
}

void CPUFlowProcessor::initialize()
{
    m_winSize = 21;
    m_levels = 6;

     m_termCriteria = cv::TermCriteria( cv::TermCriteria::EPS, 1000, 1.e-4 );

     m_minEigenValue = 1.e-4;

}

void CPUFlowProcessor::track( const std::vector< cv::Mat > &sourceImagePyramid, const std::vector< cv::Point2f > &sourcePoints, const std::vector< cv::Mat > &targetImagePyramid, std::vector< FlowTrackResult > *trackedPoints )
{
    if ( trackedPoints && !sourcePoints.empty() ) {

        std::vector< cv::Point2f > opticalPoints;
        std::vector< cv::Point2f > checkPoints;

        std::vector< unsigned char > statuses;
        std::vector< float > err;

        std::vector< unsigned char > checkStatuses;
        std::vector< float > checkErr;

        cv::calcOpticalFlowPyrLK( sourceImagePyramid, targetImagePyramid, sourcePoints, opticalPoints, statuses, err, cv::Size( m_winSize, m_winSize ),
                                            m_levels, m_termCriteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, m_minEigenValue );

        cv::calcOpticalFlowPyrLK( targetImagePyramid, sourceImagePyramid, opticalPoints, checkPoints, checkStatuses, checkErr, cv::Size( m_winSize, m_winSize ),
                                            m_levels, m_termCriteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, m_minEigenValue );

        auto rows = targetImagePyramid.front().rows;
        auto cols = targetImagePyramid.front().cols;

        trackedPoints->clear();

        for ( size_t i = 0; i < statuses.size(); ++i ) {

            auto miss = cv::norm( checkPoints[ i ] - sourcePoints[ i ] );

            if ( statuses[i] && checkStatuses[i] && miss < m_checkDistance
                 && opticalPoints[ i ].x >= 0 && opticalPoints[ i ].y >= 0
                 && opticalPoints[ i ].x < cols &&  opticalPoints[ i ].y < rows ) {

                trackedPoints->push_back( FlowTrackResult( i, opticalPoints[ i ], err[ i ], miss ) );

            }

        }

    }

}

void CPUFlowProcessor::buildImagePyramid( const CvImage &image, std::vector< cv::Mat > *imagePyramid )
{
    if ( imagePyramid )
        cv::buildOpticalFlowPyramid( image, *imagePyramid, cv::Size( m_winSize, m_winSize ), m_levels );
}

size_t CPUFlowProcessor::winSize() const
{
    return m_winSize;
}

void CPUFlowProcessor::setWinSize( const size_t value )
{
    m_winSize = value;
}

size_t CPUFlowProcessor::levels() const
{
    return m_levels;
}

void CPUFlowProcessor::setLevels( const size_t value )
{
    m_levels = value;
}

void CPUFlowProcessor::setPrecission( const double value )
{
    m_termCriteria.epsilon = value;
}

double CPUFlowProcessor::precission() const
{
    return m_termCriteria.epsilon;
}

void CPUFlowProcessor::setMinEigenValue( const double value )
{
    m_minEigenValue = value;
}

double CPUFlowProcessor::minEigenValue() const
{
    return m_minEigenValue;
}

// KeyPointProcessor
void KeyPointProcessor::extractKeypoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints )
{
    ::extractKeypoints( m_processor, image, mask, keypoints );
}

// DescriptorProcessor
void DescriptorProcessor::extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors )
{
    ::extractDescriptors( m_processor, image, keypoints, descriptors );
}

// FullProcessor
void FullProcessor::extractKeypoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints )
{
    ::extractKeypoints( m_processor, image, mask, keypoints );
}

void FullProcessor::extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors )
{
    ::extractDescriptors( m_processor, image, keypoints, descriptors );
}

void FullProcessor::extractAndCompute( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors )
{
    ::extractAndCompute( m_processor, image, mask, keypoints, descriptors );
}

// GFTTProcessor
GFTTProcessor::GFTTProcessor()
{
    initialize();
}

void GFTTProcessor::initialize()
{
    m_processor = cv::GFTTDetector::create( 1000, 1.e-3, 10. );
}

cv::Ptr< cv::GFTTDetector > GFTTProcessor::processor() const
{
    return std::dynamic_pointer_cast< cv::GFTTDetector >( m_processor );
}

void GFTTProcessor::setMaxFeatures( const int value )
{
    processor()->setMaxFeatures( value );
}

int GFTTProcessor::maxFeatures() const
{
    return processor()->getMaxFeatures();
}

// FastProcessor
FastProcessor::FastProcessor()
{
    initialize();
}

void FastProcessor::initialize()
{
    m_processor = cv::FastFeatureDetector::create( 50 );
}

cv::Ptr< cv::FastFeatureDetector > FastProcessor::processor() const
{
    return std::dynamic_pointer_cast< cv::FastFeatureDetector >( m_processor );
}

void FastProcessor::setThreshold( const int value )
{
    processor()->setThreshold( value );
}

int FastProcessor::threshold() const
{
    return processor()->getThreshold();
}

// DaisyProcessor
DaisyProcessor::DaisyProcessor()
{
    initialize();
}

void DaisyProcessor::initialize()
{
    m_processor = cv::xfeatures2d::DAISY::create();
}

// DaisyProcessor
FreakProcessor::FreakProcessor()
{
    initialize();
}

void FreakProcessor::initialize()
{
    m_processor = cv::xfeatures2d::FREAK::create();
}

// SiftProcessor
SiftProcessor::SiftProcessor()
{
    initialize();
}

void SiftProcessor::initialize()
{
    m_processor = cv::SIFT::create( 5000 );
}

// SurfProcessor
SurfProcessor::SurfProcessor()
{
    initialize();
}

void SurfProcessor::initialize()
{
    m_processor = cv::xfeatures2d::SURF::create();
}

// OrbProcessor
OrbProcessor::OrbProcessor()
{
    initialize();
}

void OrbProcessor::initialize()
{
    m_processor = cv::ORB::create( 5000 );
}

// KazeProcessor
KazeProcessor::KazeProcessor()
{
    initialize();
}

void KazeProcessor::initialize()
{
    m_processor = cv::KAZE::create();
}

// AKazeProcessor
AKazeProcessor::AKazeProcessor()
{
    initialize();
}

void AKazeProcessor::initialize()
{
    m_processor = cv::AKAZE::create();
}

// FeatureMatcherBase
FeatureMatcherBase::FeatureMatcherBase()
{
}

// DescriptorMatcherBase
double DescriptorMatcherBase::m_threshold = 0.8;

DescriptorMatcherBase::DescriptorMatcherBase()
    : FeatureMatcherBase()
{
}

cv::Mat DescriptorMatcherBase::match( const std::vector< cv::KeyPoint > &queryKeypoints, const cv::Mat &queryDescriptors,
                              const std::vector< cv::KeyPoint > &trainKeypoints, const cv::Mat &trainDescriptors, std::vector< cv::DMatch > *matches )
{
    if ( matches ) {

        matches->clear();

        if ( !queryKeypoints.empty() ) {

            std::vector< std::vector< cv::DMatch > > fwdKnnMatches;
            m_matcher->knnMatch( queryDescriptors, trainDescriptors, fwdKnnMatches, 2 );

            std::vector< cv::DMatch > fwdMatches;

            for ( size_t i = 0; i < fwdKnnMatches.size(); ++i ) {
                if ( fwdKnnMatches[ i ].size() > 1 && fwdKnnMatches[ i ][ 0 ].distance < m_threshold * fwdKnnMatches[ i ][ 1 ].distance ) {
                    fwdMatches.push_back( fwdKnnMatches[ i ][ 0 ] );
                }

            }

            std::vector< std::vector< cv::DMatch > > revKnnMatches;
            m_matcher->knnMatch( trainDescriptors, queryDescriptors, revKnnMatches, 2 );

            std::vector< cv::DMatch > revMatches;

            for ( size_t i = 0; i < revKnnMatches.size(); ++i ) {
                if ( revKnnMatches[ i ].size() > 1 && revKnnMatches[ i ][ 0 ].distance < m_threshold * revKnnMatches[ i ][ 1 ].distance ) {
                    revMatches.push_back( revKnnMatches[ i ][ 0 ] );
                }

            }

            std::vector< cv::DMatch > symMatches;

            for ( auto &i : fwdMatches ) {

                for ( auto &j : revMatches ) {

                    if ( i.queryIdx == j.trainIdx && i.trainIdx == j.queryIdx ) {
                        symMatches.push_back( i );
                        break;
                    }

                }

            }

            std::vector< cv::Point2f > points1, points2;

            points1.reserve( queryKeypoints.size() );
            points2.reserve( queryKeypoints.size() );

            for ( auto &i : symMatches ) {
                points1.push_back( queryKeypoints[ i.queryIdx ].pt );
                points2.push_back( trainKeypoints[ i.trainIdx ].pt );
            }

            if ( points1.size() > MIN_FMAT_POINTS_COUNT ) {

                std::vector< uchar > inliers( points1.size(), 0 );

                auto fMat = cv::findFundamentalMat( points1, points2, inliers, cv::FM_RANSAC );

                for ( size_t i = 0; i < inliers.size(); ++i )
                    if ( inliers[ i ] )
                        matches->push_back( symMatches[ i ] );

                return fMat;

            }

        }

    }

    return cv::Mat();

}

// FlannMatcher
BFMatcher::BFMatcher()
    : DescriptorMatcherBase()
{
    initialize();
}

void BFMatcher::initialize()
{
    m_matcher = cv::BFMatcher::create();
}

// FlannMatcher
FlannMatcher::FlannMatcher()
    : DescriptorMatcherBase()
{
    initialize();
}

void FlannMatcher::initialize()
{
    m_matcher = cv::FlannBasedMatcher::create();
}

