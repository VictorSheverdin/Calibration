#include "precompiled.h"

#include "featureprocessor.h"

// FeatureProcessorBase
FeatureProcessorBase::FeatureProcessorBase()
{
}

// FeatureProcessor
FeatureProcessor::FeatureProcessor()
{
    initialize();
}

void FeatureProcessor::initialize()
{
    m_detector = cv::GFTTDetector::create( 2000 );

    m_descriptor = cv::xfeatures2d::DAISY::create();

}

void FeatureProcessor::extract( const CvImage &image, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors )
{    
    if ( keypoints ) {

        auto uImage = image.getUMat( cv::ACCESS_READ );

        auto timeStart = std::chrono::system_clock::now();
        m_detector->detect( uImage, *keypoints );
        std::cout << "=============================" << std::endl;
        std::cout << "Feature detection time: " << std::chrono::duration_cast< std::chrono::microseconds >( std::chrono::system_clock::now() - timeStart ).count() << std::endl;

        timeStart = std::chrono::system_clock::now();
        if ( descriptors )
            m_descriptor->compute( image, *keypoints, *descriptors );
        std::cout << "Feature matching time: " << std::chrono::duration_cast< std::chrono::microseconds >( std::chrono::system_clock::now() - timeStart ).count() << std::endl;
        std::cout << "=============================" << std::endl;

    }

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
    if (matches ) {
        matches->clear();

        std::vector< std::vector< cv::DMatch > > knnMatches;
        m_matcher->knnMatch( queryDescriptors, trainDescriptors, knnMatches, 2 );

        std::vector< cv::DMatch > fwdMatches;

        for ( size_t i = 0; i < knnMatches.size(); ++i ) {
            if ( knnMatches[i].size() > 1 && knnMatches[i][0].distance < m_threshold * knnMatches[i][1].distance ) {
                fwdMatches.push_back( knnMatches[i][0] );
            }

        }
/*
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

        }*/

        std::vector< cv::DMatch > &symMatches = fwdMatches;

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


