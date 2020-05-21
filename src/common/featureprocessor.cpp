#include "precompiled.h"

#include "featureprocessor.h"

#include "defs.h"

void extractKeypoints( cv::Ptr< cv::Feature2D > processor, const CvImage &image, std::vector< cv::KeyPoint > *keypoints )
{
    if ( processor && keypoints ) {

        auto uImage = image.getUMat( cv::ACCESS_READ );
        processor->detect( uImage, *keypoints );

    }

}

void extractDescriptors( cv::Ptr< cv::Feature2D > processor, const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors )
{
    if ( processor && descriptors ) {
        processor->compute( image, keypoints, *descriptors );

    }

}

void extractAndCompute( cv::Ptr< cv::Feature2D > processor, const CvImage &image, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors )
{
    if ( processor && keypoints && descriptors ) {
        processor->detectAndCompute( image, cv::noArray(), *keypoints, *descriptors );

    }

}

// FlowProcessorBase
void FlowProcessorBase::extractPoints( const CvImage &image, std::vector< cv::Point2f > *points )
{
    if ( points )
        cv::goodFeaturesToTrack( image, *points, 25000, 1.e-1, 1. );
}

// KeyPointProcessor
void KeyPointProcessor::extractKeypoints( const CvImage &image, std::vector< cv::KeyPoint > *keypoints )
{
    ::extractKeypoints( m_processor, image, keypoints );
}

// DescriptorProcessor
void DescriptorProcessor::extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors )
{
    ::extractDescriptors( m_processor, image, keypoints, descriptors );
}

// FullProcessor
void FullProcessor::extractKeypoints( const CvImage &image, std::vector< cv::KeyPoint > *keypoints )
{
    ::extractKeypoints( m_processor, image, keypoints );
}

void FullProcessor::extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors )
{
    ::extractDescriptors( m_processor, image, keypoints, descriptors );
}

void FullProcessor::extractAndCompute( const CvImage &image, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors )
{
    ::extractAndCompute( m_processor, image, keypoints, descriptors );
}

// GFTTProcessor
GFTTProcessor::GFTTProcessor()
{
    initialize();
}

void GFTTProcessor::initialize()
{
    m_processor = cv::GFTTDetector::create( 1000, 1.e-1, 3.0, 3 );
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
    m_processor = cv::xfeatures2d::SIFT::create( 5000 );
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
    m_processor = cv::ORB::create( 5000, 1.2, 32 );
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

        std::vector< std::vector< cv::DMatch > > knnMatches;
        m_matcher->knnMatch( queryDescriptors, trainDescriptors, knnMatches, 2 );

        std::vector< cv::DMatch > fwdMatches;

        for ( size_t i = 0; i < knnMatches.size(); ++i ) {
            if ( knnMatches[ i ].size() > 1 && knnMatches[ i ][ 0 ].distance < m_threshold * knnMatches[ i ][ 1 ].distance ) {
                fwdMatches.push_back( knnMatches[ i ][ 0 ] );
            }

        }

        std::vector< cv::DMatch > revMatches;
        m_matcher->match( trainDescriptors, queryDescriptors, revMatches );

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

        std::vector< uchar > inliers( points1.size(), 0 );

        auto fMat = cv::findFundamentalMat( points1, points2, inliers, cv::FM_RANSAC );

        for ( size_t i = 0; i < inliers.size(); ++i )
            if ( inliers[ i ] )
                matches->push_back( symMatches[ i ] );

        return fMat;

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

// OpticalMatcherBase
const double OpticalMatcherBase::m_maxDistance = 1.0;
const double OpticalMatcherBase::m_errorRatio = 0.3;

// GPUOpticalMatcher
GPUOpticalMatcher::GPUOpticalMatcher()
{
    initialize();
}

void GPUOpticalMatcher::initialize()
{
    m_opticalProcessor = cv::cuda::SparsePyrLKOpticalFlow::create(/* cv::Size( 31, 31 ), 7, 100 */);
}

void download( const cv::cuda::GpuMat& d_mat, std::vector< cv::Point2f >& vec )
{
    vec.resize(d_mat.cols);
    cv::Mat mat( 1, d_mat.cols, CV_32FC2, ( void* )&vec[ 0 ] );
    d_mat.download(mat);
}

void download(const cv::cuda::GpuMat& d_mat, std::vector< uchar >& vec)
{
    vec.resize( d_mat.cols );
    cv::Mat mat( 1, d_mat.cols, CV_8UC1, ( void* )&vec[ 0 ] );
    d_mat.download( mat );
}

void download(const cv::cuda::GpuMat& d_mat, std::vector< float >& vec)
{
    vec.resize( d_mat.cols );
    cv::Mat mat( 1, d_mat.cols, CV_32FC1, ( void* )&vec[ 0 ] );
    d_mat.download( mat );
}

cv::Mat GPUOpticalMatcher::match( const CvImage &sourceImage, const std::vector< cv::KeyPoint > &sourceKeypoints,
            const CvImage &targetImage, const std::vector< cv::KeyPoint > &targetKeypoints, const cv::Mat &targetSearchMatrix,
            std::vector< cv::DMatch > *matches )
{
    if ( !matches )
        return cv::Mat();

    matches->clear();

    std::vector< cv::Point2f > sourcePoints( sourceKeypoints.size() );
    std::vector< cv::Point2f > opticalPoints;
    std::vector< cv::Point2f > checkPoints;

    for ( size_t i = 0; i < sourceKeypoints.size(); ++i )
        sourcePoints[ i ] = sourceKeypoints[ i ].pt;

    std::vector< unsigned char > statuses;
    std::vector< float > err;

    std::vector< unsigned char > checkStatuses;
    std::vector< float > checkErr;

    m_gpuSourceImage.upload( sourceImage );
    m_gpuTargetImage.upload( targetImage );
    m_gpuSourcePoints.upload( sourcePoints );

    m_opticalProcessor->calc( m_gpuSourceImage, m_gpuTargetImage, m_gpuSourcePoints, m_gpuOpticalPoints, m_gpuStatuses, m_gpuErr );
    m_opticalProcessor->calc( m_gpuTargetImage, m_gpuSourceImage, m_gpuOpticalPoints, m_gpuCheckPoints, m_gpuCheckStatuses, m_gpuCheckErr );

    download( m_gpuSourcePoints, sourcePoints );
    download( m_gpuOpticalPoints, opticalPoints );
    download( m_gpuCheckPoints, checkPoints );
    download( m_gpuStatuses, statuses );
    download( m_gpuCheckStatuses, checkStatuses );
    download( m_gpuErr, err );
    download( m_gpuCheckErr, checkErr );

    if ( !err.empty() && err.size() == checkErr.size() ) {

        float minErr;
        float maxErr;
        float minCheckErr;
        float maxCheckErr;

        minErr = err.front();
        maxErr = err.front();
        minCheckErr = checkErr.front();
        maxCheckErr = checkErr.front();

        for ( size_t i = 1; i < err.size(); ++i ) {
            minErr = std::min( minErr, err[ i ] );
            maxErr = std::max( maxErr, err[ i ] );
            minCheckErr = std::min( minCheckErr, checkErr[ i ] );
            maxCheckErr = std::max( maxCheckErr, checkErr[ i ] );
        }

        auto errDiff = maxErr - minErr;
        auto checkErrDiff = maxCheckErr - minCheckErr;

        if ( !targetKeypoints.empty() ) {

            std::vector< size_t > correspondences;

            std::vector< cv::Point2f > points1;
            std::vector< cv::Point2f > points2;

            correspondences.reserve( statuses.size() );
            points1.reserve( statuses.size() );
            points2.reserve( statuses.size() );

            auto maxErr = minErr + errDiff * m_errorRatio;
            auto maxCheckErr = minCheckErr + checkErrDiff * m_errorRatio;

            for ( size_t i = 0; i < statuses.size(); ++i ) {

                if ( statuses[i] && checkStatuses[i] && err[i] < maxErr && checkErr[i] < maxCheckErr && cv::norm( checkPoints[ i ] - sourcePoints[ i ] ) < m_maxDistance  ) {
                    points1.push_back( sourcePoints[ i ] );
                    points2.push_back( opticalPoints[ i ] );

                    correspondences.push_back( i );

                }

            }

            if ( points1.size() > MIN_TRACK_POINTS_COUNT ) {

                std::vector< uchar > inliers( points1.size(), 0 );
                auto fmat = cv::findFundamentalMat( points1, points2, inliers, cv::FM_RANSAC );

                matches->reserve( inliers.size() );

                for ( size_t i = 0; i < inliers.size(); ++i ) {

                    if ( inliers[ i ] ) {

                        auto pt = points2[ i ];

                        size_t minIndex = 0;
                        auto minDistance = cv::norm( pt - targetKeypoints.front().pt );

                        for ( int j = std::max( 0.0, pt.y - m_maxDistance );
                                        j < std::min( targetSearchMatrix.rows,
                                                      static_cast< int >( pt.y + m_maxDistance + 1 ) ); ++j  ) {

                            for ( int k = std::max( 0.0, pt.x - m_maxDistance );
                                            k < std::min( targetSearchMatrix.cols,
                                                          static_cast< int >( pt.x + m_maxDistance + 1 ) ); ++k  ) {

                                auto index = targetSearchMatrix.at< int >( j, k );

                                if ( index != -1 ) {

                                    auto distance = cv::norm( pt - targetKeypoints[ index ].pt );

                                    if ( distance < minDistance ) {
                                        minDistance = distance;
                                        minIndex = index;

                                    }

                                }

                            }

                        }

                        if ( minDistance <= m_maxDistance )
                            matches->push_back( cv::DMatch( correspondences[ i ], minIndex, minDistance ) );

                    }

                }

                return fmat;

            }


        }

    }

    return cv::Mat();

}

// CPUOpticalMatcher
void CPUOpticalMatcher::buildImagePyramid( const CvImage &image, std::vector< cv::Mat > *imagePyramid )
{
    if ( imagePyramid )
        cv::buildOpticalFlowPyramid( image, *imagePyramid, cv::Size( 21, 21 ), 3 );
}

cv::Mat CPUOpticalMatcher::match( const std::vector< cv::Mat > &sourceImagePyramid, const std::vector< cv::KeyPoint > &sourceKeypoints,
            const std::vector< cv::Mat > &targetImagePyramid, const std::vector< cv::KeyPoint > &targetKeypoints, const cv::Mat &targetSearchMatrix,
            std::vector< cv::DMatch > *matches )
{
    if ( !matches )
        return cv::Mat();

    matches->clear();

    std::vector< cv::Point2f > sourcePoints( sourceKeypoints.size() );
    std::vector< cv::Point2f > opticalPoints;
    std::vector< cv::Point2f > checkPoints;

    for ( size_t i = 0; i < sourceKeypoints.size(); ++i )
        sourcePoints[ i ] = sourceKeypoints[ i ].pt;

    std::vector< unsigned char > statuses;
    std::vector< float > err;

    std::vector< unsigned char > checkStatuses;
    std::vector< float > checkErr;

    cv::calcOpticalFlowPyrLK( sourceImagePyramid, targetImagePyramid, sourcePoints, opticalPoints, statuses, err );
    cv::calcOpticalFlowPyrLK( targetImagePyramid, sourceImagePyramid, opticalPoints, checkPoints, checkStatuses, checkErr );

    if ( !err.empty() && err.size() == checkErr.size() ) {

        float minErr;
        float maxErr;
        float minCheckErr;
        float maxCheckErr;

        minErr = err.front();
        maxErr = err.front();
        minCheckErr = checkErr.front();
        maxCheckErr = checkErr.front();

        for ( size_t i = 1; i < err.size(); ++i ) {
            minErr = std::min( minErr, err[ i ] );
            maxErr = std::max( maxErr, err[ i ] );
            minCheckErr = std::min( minCheckErr, checkErr[ i ] );
            maxCheckErr = std::max( maxCheckErr, checkErr[ i ] );
        }

        auto errDiff = maxErr - minErr;
        auto checkErrDiff = maxCheckErr - minCheckErr;

        if ( !targetKeypoints.empty() ) {

            std::vector< size_t > correspondences;

            std::vector< cv::Point2f > points1;
            std::vector< cv::Point2f > points2;

            correspondences.reserve( statuses.size() );
            points1.reserve( statuses.size() );
            points2.reserve( statuses.size() );

            auto maxErr = minErr + errDiff * m_errorRatio;
            auto maxCheckErr = minCheckErr + checkErrDiff * m_errorRatio;

            for ( size_t i = 0; i < statuses.size(); ++i ) {

                if ( statuses[i] && checkStatuses[i] && err[i] < maxErr && checkErr[i] < maxCheckErr && cv::norm( checkPoints[ i ] - sourcePoints[ i ] ) < m_maxDistance  ) {
                    points1.push_back( sourcePoints[ i ] );
                    points2.push_back( opticalPoints[ i ] );

                    correspondences.push_back( i );

                }

            }

            if ( points1.size() > MIN_TRACK_POINTS_COUNT ) {

                std::vector< uchar > inliers( points1.size(), 0 );
                auto fmat = cv::findFundamentalMat( points1, points2, inliers, cv::FM_RANSAC );

                matches->reserve( inliers.size() );

                for ( size_t i = 0; i < inliers.size(); ++i ) {

                    if ( inliers[ i ] ) {

                        auto pt = points2[ i ];

                        size_t minIndex = 0;
                        auto minDistance = cv::norm( pt - targetKeypoints.front().pt );

                        for ( int j = std::max( 0.0, pt.y - m_maxDistance );
                                        j < std::min( targetSearchMatrix.rows,
                                                      static_cast< int >( pt.y + m_maxDistance + 1 ) ); ++j  ) {

                            for ( int k = std::max( 0.0, pt.x - m_maxDistance );
                                            k < std::min( targetSearchMatrix.cols,
                                                          static_cast< int >( pt.x + m_maxDistance + 1 ) ); ++k  ) {

                                auto index = targetSearchMatrix.at< int >( j, k );

                                if ( index != -1 ) {

                                    auto distance = cv::norm( pt - targetKeypoints[ index ].pt );

                                    if ( distance < minDistance ) {
                                        minDistance = distance;
                                        minIndex = index;

                                    }

                                }

                            }

                        }

                        if ( minDistance <= m_maxDistance )
                            matches->push_back( cv::DMatch( correspondences[ i ], minIndex, minDistance ) );

                    }

                }

                return fmat;

            }


        }

    }

    return cv::Mat();

}

