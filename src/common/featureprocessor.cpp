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

// FeatureProcessorBase
FeatureProcessorBase::FeatureProcessorBase()
{
    initialize();
}

void FeatureProcessorBase::initialize()
{
    m_ransacReprojectionThreshold = 3.0;
    m_ransacConfidence = 0.99;
}

double FeatureProcessorBase::ransacReprojectionThreshold() const
{
    return m_ransacReprojectionThreshold;
}

void FeatureProcessorBase::setRansacReprojectionThreshold( const double &value )
{
    m_ransacReprojectionThreshold = value;
}

double FeatureProcessorBase::ransacConfidence() const
{
    return m_ransacConfidence;
}

void FeatureProcessorBase::setRansacConfidence( const double &value )
{
    m_ransacConfidence = value;
}

cv::Mat FeatureProcessorBase::epipolarTest( const std::vector< cv::Point2f > &sourcePoints, const std::vector< cv::Point2f > &targetPoints, std::vector< size_t > *inliers )
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

// FlowProcessor
FlowProcessor::FlowProcessor()
{
    initialize();
}

void FlowProcessor::initialize()
{
    m_checkDistance = 1.0;
    m_extractPrecision = 1.e-2;
    m_extractDistance = 10.;
    m_blockSize = 3;
}

void FlowProcessor::extractPoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::Point2f > *points, const size_t count )
{
    if ( points ) {
        cv::Mat gray;
        cv::cvtColor( image, gray, cv::COLOR_BGR2GRAY );

        cv::goodFeaturesToTrack( gray, *points, count, m_extractPrecision, m_extractDistance, mask, m_blockSize );

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

double FlowProcessor::checkDistance() const
{
    return m_checkDistance;
}

void FlowProcessor::setCheckDistance( const double value )
{
    m_checkDistance = value;
}

void FlowProcessor::setExtractionDistance( const double value )
{
    m_extractDistance = value;
}

double FlowProcessor::extractionDistance() const
{
    return m_extractDistance;
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
    m_winSize = 17;
    m_levels = 3;

    m_termCriteria = cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1.e-2 );

    m_minEigenValue = 1.e-2;

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

void GFTTProcessor::setQualityLevel( double value )
{
    processor()->setQualityLevel( value );
}

double GFTTProcessor::qualityLevel() const
{
    return processor()->getQualityLevel();
}

void GFTTProcessor::setMinDistance( double value )
{
    processor()->setMinDistance( value );
}

double GFTTProcessor::minDistance() const
{
    return processor()->getMinDistance();
}

void GFTTProcessor::setBlockSize( int value )
{
    processor()->setBlockSize( value );
}

int GFTTProcessor::blockSize() const
{
    return processor()->getBlockSize();
}

void GFTTProcessor::setHarrisDetector( bool value )
{
    processor()->setHarrisDetector( value );
}

bool GFTTProcessor::harrisDetector() const
{
    return processor()->getHarrisDetector();
}

void GFTTProcessor::setK( double value )
{
    processor()->setK( value );
}

double GFTTProcessor::k() const
{
    return processor()->getK();
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

void FastProcessor::setNonmaxSuppression( bool value )
{
    processor()->setNonmaxSuppression( value );
}

bool FastProcessor::nonmaxSuppression()
{
    return processor()->getNonmaxSuppression();
}

void FastProcessor::setType( cv::FastFeatureDetector::DetectorType value )
{
    processor()->setType( value );
}

cv::FastFeatureDetector::DetectorType FastProcessor::type()
{
    return processor()->getType();
}

// SuperGlueProcessor
SuperGlueProcessor::SuperGlueProcessor( const std::string &detectorModelFile, const std::string &matcherModelFile )
{
    initialize( detectorModelFile, matcherModelFile );
}

void SuperGlueProcessor::initialize( const std::string &detectorModelFile, const std::string &matcherModelFile )
{
    marker::StreamLogger logger( std::cout, marker::StreamLogger::Severity::kWARNING );

    marker::KeypointSelector::Config cfg;

    cfg.border = 10;
    cfg.score_threshold = .05;

    _matcherThreshold = .25;

    _detector = std::make_unique< marker::SuperPointDetector >( detectorModelFile, logger );
    _keypointSelector = std::make_unique< marker::KeypointSelector >( cfg, _detector->scores_shape() );
    _matcher = std::make_unique< marker::SuperGlueMatcher >( matcherModelFile, logger );
}

void SuperGlueProcessor::setMatchingThreshold( const double value )
{
    _matcherThreshold = value;
}

int SuperGlueProcessor::matchingThreshold() const
{
    return _matcherThreshold;
}

void SuperGlueProcessor::extract( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors, const size_t count )
{
    CV_Assert( image.channels() == 1 );
    CV_Assert( count <= _maxPointsCount );

    _detector->detect( image );
    auto& scoreMap = _detector->score_map();
    auto& descrMap = _detector->descriptor_map();

    auto keypointSet = marker::KeypointSet::create();

    _keypointSelector->select( scoreMap, mask, count, keypointSet );

    auto descriptorSet = marker::sample_descriptors( descrMap, keypointSet );

    keypointSet.download( *keypoints );

    descriptorSet.download( *descriptors );

}

void SuperGlueProcessor::match( const cv::Size &imageSize1, const cv::Size &imageSize2, const std::vector< cv::KeyPoint > &keypoints1, const std::vector< cv::KeyPoint > &keypoints2, const cv::Mat &descriptors1, const cv::Mat &descriptors2, std::vector< cv::DMatch > *matches )
{
    auto keypointSet1 = marker::KeypointSet::create();
    auto keypointSet2 = marker::KeypointSet::create();

    keypointSet1.image_size = imageSize1;
    keypointSet2.image_size = imageSize2;

    keypointSet1.upload( keypoints1 );
    keypointSet2.upload( keypoints2 );

    auto descriptorSet1 = marker::DescriptorSet::create();
    auto descriptorSet2 = marker::DescriptorSet::create();

    descriptorSet1.upload( descriptors1 );
    descriptorSet2.upload( descriptors2 );

    _matcher->match( keypointSet1, keypointSet2, descriptorSet1, descriptorSet2 );
    auto& match_table = _matcher->output();

    cv::Mat scores;

    match_table.download( scores );

    matches->clear();

    for( int i = 0; i < scores.rows - 1; ++i ) {

        auto row_begin = scores.ptr< float >( i );
        auto row_end = row_begin + scores.cols;
        auto max_score = std::max_element( row_begin, row_end );

        if ( max_score < row_end - 1 && *max_score > _matcherThreshold ) {

            int j = max_score - row_begin;
            float d = 1.f - *max_score;
            matches->emplace_back( i, j, d );

        }

    }

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

// DescriptorMatcher
double DescriptorMatcher::m_threshold = 0.8;

DescriptorMatcher::DescriptorMatcher()
    : FeatureMatcherBase()
{
}

cv::Mat DescriptorMatcher::match( const std::vector< cv::KeyPoint > &queryKeypoints, const cv::Mat &queryDescriptors,
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
                        matches->push_back( i );
                        break;
                    }

                }

            }


        }

    }

    return cv::Mat();

}

// FlannMatcher
BFMatcher::BFMatcher()
    : DescriptorMatcher()
{
    initialize();
}

void BFMatcher::initialize()
{
    m_matcher = cv::BFMatcher::create();
}

// FlannMatcher
FlannMatcher::FlannMatcher()
    : DescriptorMatcher()
{
    initialize();
}

void FlannMatcher::initialize()
{
    m_matcher = cv::FlannBasedMatcher::create();
}

