#include "precompiled.h"

#include "featureprocessor.h"

#include <opencv2/cudaoptflow.hpp>

// KeyPointProcessor
void KeyPointProcessor::extractKeypoints( const CvImage &image, std::vector< cv::KeyPoint > *keypoints )
{
    if ( keypoints ) {

        auto uImage = image.getUMat( cv::ACCESS_READ );
        m_processor->detect( uImage, *keypoints );

    }

}

// DescriptorProcessor
void DescriptorProcessor::extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors )
{
    if ( descriptors ) {
        m_processor->compute( image, keypoints, *descriptors );

    }

}

// GFTTProcessor
GFTTProcessor::GFTTProcessor()
{
    initialize();
}

void GFTTProcessor::initialize()
{
    m_processor = cv::GFTTDetector::create();
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

// DaisyProcessor
DaisyProcessor::DaisyProcessor()
{
    initialize();
}

void DaisyProcessor::initialize()
{
    m_processor = cv::xfeatures2d::DAISY::create();
}

// FeatureMatcherBase
FeatureMatcherBase::FeatureMatcherBase()
{
}

// FeatureMatcher
double FeatureMatcher::m_threshold = 0.8;

FeatureMatcher::FeatureMatcher()
{
    initialize();
}

void FeatureMatcher::initialize()
{
    m_matcher = cv::FlannBasedMatcher::create();
}

cv::Mat FeatureMatcher::match( std::vector< cv::KeyPoint > &queryKeypoints, const cv::Mat &queryDescriptors,
                              std::vector< cv::KeyPoint > &trainKeypoints, const cv::Mat &trainDescriptors, std::vector< cv::DMatch > *matches )
{
    if ( matches ) {
        matches->clear();

        std::vector< std::vector< cv::DMatch > > knnMatches;
        m_matcher->knnMatch( queryDescriptors, trainDescriptors, knnMatches, 2 );

        std::vector< cv::DMatch > fwdMatches;

        for ( size_t i = 0; i < knnMatches.size(); ++i ) {
            if ( knnMatches[i].size() > 1 && knnMatches[i][0].distance < m_threshold * knnMatches[i][1].distance ) {
                fwdMatches.push_back( knnMatches[i][0] );
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

// OpticalMatcher
const double OpticalMatcher::m_maxDistance = 1.0;

OpticalMatcher::OpticalMatcher()
{
    initialize();
}

void OpticalMatcher::initialize()
{
    m_opticalProcessor = cv::cuda::SparsePyrLKOpticalFlow::create();
}

void download( const cv::cuda::GpuMat& d_mat, std::vector< cv::Point2f >& vec )
{
    vec.resize(d_mat.cols);
    cv::Mat mat( 1, d_mat.cols, CV_32FC2, (void*)&vec[0] );
    d_mat.download(mat);
}

void download(const cv::cuda::GpuMat& d_mat, std::vector< uchar >& vec)
{
    vec.resize( d_mat.cols );
    cv::Mat mat( 1, d_mat.cols, CV_8UC1, (void*)&vec[0] );
    d_mat.download( mat );
}

void download(const cv::cuda::GpuMat& d_mat, std::vector< float >& vec)
{
    vec.resize( d_mat.cols );
    cv::Mat mat( 1, d_mat.cols, CV_32FC1, (void*)&vec[0] );
    d_mat.download( mat );
}

cv::Mat OpticalMatcher::match( const CvImage &sourceImage, std::vector< cv::KeyPoint > &sourceKeypoints,
            const CvImage &targetImage, std::vector< cv::KeyPoint > &targetKeypoints,
            std::vector< cv::DMatch > *matches )
{
    if ( !matches )
        return cv::Mat();

    matches->clear();

    std::vector< cv::Point2f > sourcePoints( sourceKeypoints.size() );
    std::vector< cv::Point2f > opticalPoints;

#pragma omp parallel for
    for ( size_t i = 0; i < sourceKeypoints.size(); ++i )
        sourcePoints[ i ] = sourceKeypoints[ i ].pt;

    std::vector< unsigned char > statuses;
    std::vector< float > err;

    m_gpuSourceImage.upload( sourceImage );
    m_gpuTargetImage.upload( targetImage );
    m_gpuSourcePoints.upload( sourcePoints );

    m_opticalProcessor->calc( m_gpuSourceImage, m_gpuTargetImage, m_gpuSourcePoints, m_gpuOpticalPoints, m_gpuStatuses, m_gpuErr );

    download( m_gpuSourcePoints, sourcePoints );
    download( m_gpuOpticalPoints, opticalPoints );
    download( m_gpuStatuses, statuses );
    download( m_gpuErr, err );

/*    auto sourceUImage = sourceImage.getUMat( cv::ACCESS_READ );
    auto targetUImage = targetImage.getUMat( cv::ACCESS_READ );

    cv::calcOpticalFlowPyrLK( sourceUImage, targetUImage, sourceUPoints, opticalPoints, statuses, err );*/

    if ( !err.empty() ) {

        float minErr;
        float maxErr;

        minErr = err.front();
        maxErr = err.front();

        for ( size_t i = 1; i < err.size(); ++i ) {
            minErr = std::min( minErr, err[ i ] );
            maxErr = std::max( maxErr, err[ i ] );
        }

        auto errDiff = maxErr - minErr;

        if ( !targetKeypoints.empty() ) {

            std::vector< size_t > correspondences;

            std::vector< cv::Point2f > points1;
            std::vector< cv::Point2f > points2;

            correspondences.reserve( statuses.size() );
            points1.reserve( statuses.size() );
            points2.reserve( statuses.size() );

            auto maxErr = minErr + errDiff / 3.f;

            for ( size_t i = 0; i < statuses.size(); ++i ) {

                if ( statuses[i] && err[i] < maxErr ) {
                    points1.push_back( sourcePoints[ i ] );
                    points2.push_back( opticalPoints[ i ] );

                    correspondences.push_back( i );

                }

            }

            if ( points1.size() > m_minPointsCount ) {

                std::vector< uchar > inliers( points1.size(), 0 );
                auto fmat = cv::findFundamentalMat( points1, points2, inliers, cv::FM_RANSAC );

                matches->reserve( inliers.size() );

                using KeypointsRow = std::vector< int >;
                using KeypointsMatrix = std::vector< KeypointsRow >;

                KeypointsMatrix keypointsMatrix( targetImage.rows, KeypointsRow( targetImage.cols, -1 ) );

                // Acceleration matrix
#pragma omp parallel for
                for ( size_t i = 0; i < targetKeypoints.size(); ++i )
                    keypointsMatrix[ targetKeypoints[ i ].pt.y ][ targetKeypoints[ i ].pt.x ] = i;

                for ( size_t i = 0; i < inliers.size(); ++i ) {

                    if ( inliers[ i ] ) {

                        auto pt = points2[ i ];

                        size_t minIndex = 0;
                        auto minDistance = cv::norm( pt - targetKeypoints.front().pt );

                        for ( size_t j = std::max( 0.0, pt.y - m_maxDistance );
                                        j < std::min( keypointsMatrix.size(),
                                                      static_cast< size_t >( pt.y + m_maxDistance + 1 ) ); ++j  ) {

                            for ( size_t k = std::max( 0.0, pt.x - m_maxDistance );
                                            k < std::min( keypointsMatrix.at( j ).size(),
                                                          static_cast< size_t >( pt.x + m_maxDistance + 1 ) ); ++k  ) {

                                auto index = keypointsMatrix[ j ][ k ];

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


