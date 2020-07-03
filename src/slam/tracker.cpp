#include "src/common/precompiled.h"

#include "tracker.h"

#include "frame.h"

namespace slam {

// FlowTracker
void FlowTracker::prepareStereoPoints( const FlowFramePtr &frame, std::vector< cv::Point2f > *points, std::map< size_t, cv::Point2f > *trackedMap )
{
    if ( points && trackedMap ) {

        points->clear();
        trackedMap->clear();

        auto flowPoints = frame->flowPoints();

        points->reserve( flowPoints.size() );

        for ( auto &i : flowPoints ) {
            if ( i ) {
                points->push_back( i->point() );
                auto stereoPoint = i->stereoPoint();
                if ( stereoPoint ) {
                    ( *trackedMap )[ points->size() - 1 ] = stereoPoint->point();
                }

            }

        }

    }

}

void FlowTracker::prepareConsecutivePoints( const FlowFramePtr &frame, std::vector< cv::Point2f > *points, std::map< size_t, cv::Point2f > *trackedMap )
{
    if ( points && trackedMap ) {

        points->clear();

        auto flowPoints = frame->flowPoints();

        points->reserve( flowPoints.size() );

        for ( auto &i : flowPoints ) {
            if ( i ) {
                points->push_back( i->point() );
                auto nextPoint = i->nextPoint();
                if ( nextPoint ) {
                    ( *trackedMap )[ points->size() - 1 ] = nextPoint->point();
                }

            }

        }

    }

}

// GPUFlowTracker
void GPUFlowTracker::buildPyramid( FlowFrame *frame )
{
}

void GPUFlowTracker::extractPoints( FlowFrame *frame )
{
    if ( frame ) {

        std::vector< cv::Point2f > points;

        m_pointsProcessor.extractPoints( frame->image(), &points );

        frame->setExtractedPoints( points );

    }

}

cv::Mat GPUFlowTracker::match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap )
{
    if ( trackedMap ) {

        std::vector< cv::Point2f > points;
        prepareStereoPoints( frame1, &points, trackedMap );

        auto fmat = m_pointsProcessor.track( frame1->image(), points, frame2->image(), trackedMap );

        return fmat;
    }

    return cv::Mat();

}

cv::Mat GPUFlowTracker::track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap )
{
    if ( trackedMap ) {

        std::vector< cv::Point2f > points;
        prepareConsecutivePoints( frame1, &points, trackedMap );

        auto fmat = m_pointsProcessor.track( frame1->image(), points, frame2->image(), trackedMap );

        return fmat;
    }

    return cv::Mat();

}

size_t GPUFlowTracker::count() const
{
    return m_pointsProcessor.count();
}

void GPUFlowTracker::setCount( const size_t value )
{
    m_pointsProcessor.setCount( value );
}

// CPUFlowTracker
void CPUFlowTracker::buildPyramid( FlowFrame *frame )
{
    if ( frame ) {

        std::vector< cv::Mat > imagePyramid;

        m_pointsProcessor.buildImagePyramid( frame->image(), &imagePyramid );

        frame->setImagePyramid( imagePyramid );

    }

}

void CPUFlowTracker::extractPoints( FlowFrame *frame )
{
    if ( frame ) {

        std::vector< cv::Point2f > points;

        m_pointsProcessor.extractPoints( frame->image(), &points );

        frame->setExtractedPoints( points );

    }

}

cv::Mat CPUFlowTracker::match( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap )
{
    if ( frame1 && frame2 && trackedMap ) {

        if ( frame1->imagePyramid().empty() )
            frame1->buildPyramid();

        if ( frame2->imagePyramid().empty() )
            frame2->buildPyramid();

        std::vector< cv::Point2f > points;
        prepareStereoPoints( frame1, &points, trackedMap );

        auto fmat = m_pointsProcessor.track( frame1->imagePyramid(), points, frame2->imagePyramid(), trackedMap );

        return fmat;
    }

    return cv::Mat();
}

cv::Mat CPUFlowTracker::track( const FlowFramePtr &frame1, const FlowFramePtr &frame2, std::map< size_t, cv::Point2f > *trackedMap )
{
    if ( frame1 && frame2 && trackedMap ) {

        if ( frame1->imagePyramid().empty() )
            frame1->buildPyramid();

        if ( frame2->imagePyramid().empty() )
            frame2->buildPyramid();

        std::vector< cv::Point2f > points;
        prepareConsecutivePoints( frame1, &points, trackedMap );

        auto fmat = m_pointsProcessor.track( frame1->imagePyramid(), points, frame2->imagePyramid(), trackedMap );

        return fmat;
    }

    return cv::Mat();

}

size_t CPUFlowTracker::count() const
{
    return m_pointsProcessor.count();
}

void CPUFlowTracker::setCount( const size_t value )
{
    m_pointsProcessor.setCount( value );
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
