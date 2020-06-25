#include "src/common/precompiled.h"

#include "tracker.h"

#include "frame.h"

namespace slam {

// GPUFlowTracker
void GPUFlowTracker::prepareFrame( FlowFrame *frame )
{
    if ( frame ) {

        std::vector< cv::Point2f > points;

        m_pointsProcessor.extractPoints( frame->image(), &points );

        frame->setExtractedPoints( points );

    }

}

cv::Mat GPUFlowTracker::match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::vector< size_t > *trackedIndexes, std::vector< cv::Point2f > *trackedPoints )
{
    if ( trackedIndexes && trackedPoints ) {

        auto flowPoints = frame1->flowPoints();

        std::vector< cv::Point2f > points;
        points.reserve( flowPoints.size() );

        for ( auto &i : flowPoints )
            if ( i )
                points.push_back( i->point() );

        auto fmat = m_pointsProcessor.track( frame1->image(), points, frame2->image(), trackedIndexes, trackedPoints );

        return fmat;
    }

    return cv::Mat();
}

// CPUFlowTracker
void CPUFlowTracker::prepareFrame( FlowFrame *frame )
{
    if ( frame ) {

        std::vector< cv::Point2f > points;

        m_pointsProcessor.extractPoints( frame->image(), &points );

        frame->setExtractedPoints( points );

        std::vector< cv::Mat > imagePyramid;

        m_pointsProcessor.buildImagePyramid( frame->image(), &imagePyramid );

        frame->setImagePyramid( imagePyramid );

    }

}

cv::Mat CPUFlowTracker::match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::vector<size_t> *trackedIndexes, std::vector<cv::Point2f> *trackedPoints )
{
    if ( trackedIndexes && trackedPoints ) {

        auto flowPoints = frame1->flowPoints();

        std::vector< cv::Point2f > points;
        points.reserve( flowPoints.size() );

        for ( auto &i : flowPoints )
            if ( i )
                points.push_back( i->point() );

        auto fmat = m_pointsProcessor.track( frame1->imagePyramid(), points, frame2->imagePyramid(), trackedIndexes, trackedPoints );

        return fmat;
    }

    return cv::Mat();
}

// OpticalTrackerBase
void OpticalTrackerBase::setMaxFeatures( const int value )
{
    m_keypointProcessor.setMaxFeatures( value );
}

int OpticalTrackerBase::maxFeatures()
{
    return m_keypointProcessor.maxFeatures();
}

void OpticalTrackerBase::prepareFrame( FeatureFrame *frame )
{
    if ( frame ) {

        std::vector< cv::KeyPoint > keypoints;

        auto &image = frame->image();

        m_keypointProcessor.extractKeypoints( image, &keypoints );

        cv::Mat searchMatrix( image.rows, image.cols, CV_32S, -1 );

        for ( size_t i = 0; i < keypoints.size(); ++i )
            searchMatrix.at< int >( keypoints[ i ].pt.y, keypoints[ i ].pt.x ) = i;

        frame->setKeyPoints( keypoints );
        frame->setSearchMatrix( searchMatrix );

    }

}

// GPUOpticalTracker
cv::Mat GPUOpticalTracker::match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches )
{
    if ( frame1 && frame2 ) {

        auto trackPoints = frame1->featurePoints();

        std::vector< cv::KeyPoint > frame1Keypoints;
        frame1Keypoints.reserve( trackPoints.size() );

        for ( auto &i : trackPoints )
            frame1Keypoints.push_back( i->keyPoint() );

        return m_opticalMatcher.match( frame1->image(), frame1Keypoints, frame2->image(), frame2->keyPoints(), frame2->searchMatrix(), matches );

    }

    return cv::Mat();

}

// CPUOpticalTracker
void CPUOpticalTracker::prepareFrame( FeatureFrame *frame )
{
    OpticalTrackerBase::prepareFrame( frame );

    if ( frame ) {
        std::vector< cv::Mat > imagePyramid;

        m_opticalMatcher.buildImagePyramid( frame->image(), &imagePyramid );

        frame->setImagePyramid( imagePyramid );

    }

}

cv::Mat CPUOpticalTracker::match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches )
{
    if ( frame1 && frame2 && matches ) {

        auto trackPoints = frame1->featurePoints();

        std::vector< cv::KeyPoint > frame1Keypoints;
        frame1Keypoints.reserve( trackPoints.size() );

        for ( auto &i : trackPoints )
            frame1Keypoints.push_back( i->keyPoint() );

        return m_opticalMatcher.match( frame1->imagePyramid(), frame1Keypoints, frame2->imagePyramid(), frame2->keyPoints(), frame2->searchMatrix(), matches );

    }

    return cv::Mat();

}

// DescriptorTracker
bool DescriptorTracker::selectKeypoints( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors )
{
    if ( frame1 && frame2 && keypoints && descriptors ) {

        auto trackPoints = frame1->featurePoints();

        keypoints->clear();

        *descriptors = cv::Mat( std::min( static_cast< int >( trackPoints.size() ), frame1->descriptors().rows ), frame1->descriptors().cols, frame1->descriptors().type() );

        for ( size_t i = 0; i < trackPoints.size(); ++i ) {

            auto point = trackPoints[ i ];

            keypoints->push_back( point->keyPoint() );

            if ( descriptors->rows > static_cast< int >( i ) )
                point->descriptor().copyTo( descriptors->row( i ) );

        }

        return true;

    }

    return false;
}

// FullTracker
void FullTracker::prepareFrame( FeatureFrame *frame )
{
    if ( frame ) {
        std::vector< cv::KeyPoint > keypoints;
        cv::Mat descriptors;

        m_descriptorProcessor->extractAndCompute( frame->image(), &keypoints, &descriptors );

        frame->setKeyPoints( keypoints );
        frame->setDescriptors( descriptors );

    }

}

// SiftTracker
SiftTracker::SiftTracker()
{
    initialize();
}

void SiftTracker::initialize()
{
    m_descriptorProcessor = std::unique_ptr< FullProcessor >( new SiftProcessor() );
}

cv::Mat SiftTracker::match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches )
{
    if ( frame1 && frame2 ) {

        std::vector< cv::KeyPoint > frame1Keypoints;
        cv::Mat frame1Descriptors ;

        if ( selectKeypoints( frame1, frame2, &frame1Keypoints, &frame1Descriptors ) )
            return m_featuresMatcher.match( frame1Keypoints, frame1Descriptors, frame2->keyPoints(), frame2->descriptors(), matches );

    }

    return cv::Mat();

}

// OrbTracker
OrbTracker::OrbTracker()
{
    initialize();
}

void OrbTracker::initialize()
{
    m_descriptorProcessor = std::unique_ptr< FullProcessor >( new OrbProcessor() );
}

cv::Mat OrbTracker::match( const FeatureFramePtr &frame1, const FeatureFramePtr &frame2, std::vector< cv::DMatch > *matches )
{
    if ( frame1 && frame2 ) {

        std::vector< cv::KeyPoint > frame1Keypoints;
        cv::Mat frame1Descriptors ;

        if ( selectKeypoints( frame1, frame2, &frame1Keypoints, &frame1Descriptors ) )
            return m_featuresMatcher.match( frame1Keypoints, frame1Descriptors, frame2->keyPoints(), frame2->descriptors(), matches );

    }

    return cv::Mat();

}

}
