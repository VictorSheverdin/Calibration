#include "src/common/precompiled.h"

#include "tracker.h"

#include "frame.h"

namespace slam {

double FlowTracker::extractPrecision() const
{
    return m_pointsProcessor->extractPrecision();
}

void FlowTracker::setExtractPrecision( const double value )
{
    m_pointsProcessor->setExtractPrecision( value );
}

size_t FlowTracker::winSize() const
{
    return m_pointsProcessor->winSize();
}

void FlowTracker::setWinSize( const size_t value )
{
    m_pointsProcessor->setWinSize( value );
}

size_t FlowTracker::levels() const
{
    return m_pointsProcessor->levels();
}

void FlowTracker::setLevels( const size_t value )
{
    m_pointsProcessor->setLevels( value );
}

// GPUFlowTracker
GPUFlowTracker::GPUFlowTracker()
{
    initialize();
}

void GPUFlowTracker::initialize()
{
    m_pointsProcessor = std::unique_ptr< FlowProcessor >( new GPUFlowProcessor() );
}

void GPUFlowTracker::buildPyramid( FlowFrame *frame )
{
}

void GPUFlowTracker::extractPoints( FlowKeyFrame *frame )
{
    if ( frame ) {
        std::vector< cv::Point2f > points;

        auto extractPointsCount = static_cast< int >( frame->desiredPointsCount() ) - static_cast< int >( frame->framePointsCount() );

        if ( extractPointsCount > 0 ) {

            frame->createMask();

            processor()->extractPoints( frame->image(), frame->mask(), &points, extractPointsCount, frame->pointsInterval() );

            frame->addFramePoints( points );

        }

    }

}

cv::Mat GPUFlowTracker::track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::vector< FlowTrackResult > *trackedPoints )
{
    if ( frame1 && frame2 && trackedPoints ) {

        std::vector< cv::Point2f > points;

        auto flowPoints = frame1->flowPoints();

        points.reserve( flowPoints.size() );

        for ( auto &i : flowPoints )
            if ( i )
                points.push_back( i->point() );

        std::vector< FlowTrackResult > flowResults;

        processor()->track( frame1->image(), points, frame2->image(), &flowResults );

        std::vector< cv::Point2f > sourcePoints, targetPoints;
        sourcePoints.reserve( flowResults.size() );
        targetPoints.reserve( flowResults.size() );

        for ( auto &i : flowResults ) {
            sourcePoints.push_back( points[ i.index ]);
            targetPoints.push_back( i );
        }

        std::vector< int > epipolarIndexes;

        auto fmat = processor()->epiTest( sourcePoints, targetPoints, &epipolarIndexes );

        trackedPoints->clear();

        for ( auto &i : epipolarIndexes )
            trackedPoints->push_back( flowResults[ i ] );

        return fmat;

    }

    return cv::Mat();

}

GPUFlowProcessor *GPUFlowTracker::processor() const
{
    return dynamic_cast< GPUFlowProcessor* >( m_pointsProcessor.get() );
}

// CPUFlowTracker
CPUFlowTracker::CPUFlowTracker()
{
    initialize();
}

void CPUFlowTracker::initialize()
{
    m_pointsProcessor = std::unique_ptr< FlowProcessor >( new CPUFlowProcessor() );
}

void CPUFlowTracker::buildPyramid( FlowFrame *frame )
{
    if ( frame ) {
        std::vector< cv::Mat > imagePyramid;

        processor()->buildImagePyramid( frame->image(), &imagePyramid );

        frame->setImagePyramid( imagePyramid );

    }

}

void CPUFlowTracker::extractPoints( FlowKeyFrame *frame )
{
    if ( frame ) {
        std::vector< cv::Point2f > points;

        auto extractPointsCount = static_cast< int >( frame->desiredPointsCount() ) - static_cast< int >( frame->framePointsCount() );

        if ( extractPointsCount > 0 ) {

            frame->createMask();

            processor()->extractPoints( frame->image(), frame->mask(), &points, extractPointsCount, frame->pointsInterval() );

            frame->addFramePoints( points );

        }

    }

}

cv::Mat CPUFlowTracker::track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::vector< FlowTrackResult > *trackedPoints )
{
    if ( frame1 && frame2 && trackedPoints ) {

        if ( frame1->imagePyramid().empty() )
            frame1->buildPyramid();

        if ( frame2->imagePyramid().empty() )
            frame2->buildPyramid();

        std::vector< cv::Point2f > points;

        auto flowPoints = frame1->flowPoints();

        points.reserve( flowPoints.size() );

        for ( auto &i : flowPoints )
            if ( i )
                points.push_back( i->point() );

        std::vector< FlowTrackResult > flowResults;

        processor()->track( frame1->imagePyramid(), points, frame2->imagePyramid(), &flowResults );

        std::vector< cv::Point2f > sourcePoints, targetPoints;
        sourcePoints.reserve( flowResults.size() );
        targetPoints.reserve( flowResults.size() );

        for ( auto &i : flowResults ) {
            sourcePoints.push_back( points[ i.index ]);
            targetPoints.push_back( i );
        }

        std::vector< int > epipolarIndexes;

        auto fmat = processor()->epiTest( sourcePoints, targetPoints, &epipolarIndexes );

        trackedPoints->clear();

        for ( auto &i : epipolarIndexes )
            trackedPoints->push_back( flowResults[ i ] );

        return fmat;

    }

    return cv::Mat();

}

CPUFlowProcessor *CPUFlowTracker::processor() const
{
    return dynamic_cast< CPUFlowProcessor* >( m_pointsProcessor.get() );
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
void FullTracker::extractKeypoints( FeatureFrame *frame )
{
    if ( frame ) {

        std::vector< cv::KeyPoint > keypoints;
        cv::Mat descriptors;

        frame->createMask();

        m_descriptorProcessor->extractAndCompute( frame->image(), frame->mask(), &keypoints, &descriptors );

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
