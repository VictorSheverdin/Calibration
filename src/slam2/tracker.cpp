#include "src/common/precompiled.h"

#include "tracker.h"

#include "track.h"

#include "frame.h"

#include "framepoint.h"

namespace slam2 {

// FlowTracker
double FlowTracker::extractPrecision() const
{
    return _pointsProcessor->extractPrecision();
}

void FlowTracker::setExtractPrecision( const double value )
{
    _pointsProcessor->setExtractPrecision( value );
}

void FlowTracker::setExtractionDistance( const double value )
{
    _pointsProcessor->setExtractionDistance( value );
}

double FlowTracker::extractionDistance() const
{
    return _pointsProcessor->extractionDistance();
}

size_t FlowTracker::winSize() const
{
    return _pointsProcessor->winSize();
}

void FlowTracker::setWinSize( const size_t value )
{
    _pointsProcessor->setWinSize( value );
}

size_t FlowTracker::levels() const
{
    return _pointsProcessor->levels();
}

void FlowTracker::setLevels( const size_t value )
{
    _pointsProcessor->setLevels( value );
}

// CPUFlowTracker
CPUFlowTracker::CPUFlowTracker()
{
    initialize();
}

void CPUFlowTracker::initialize()
{
    _pointsProcessor = std::unique_ptr< FlowProcessor >( new CPUFlowProcessor() );
}

void CPUFlowTracker::prepareFrame( ProcFrame *frame )
{
    if ( frame->imagePyramid().empty() ) {
        std::vector< cv::Mat > pyramid;
        processor()->buildImagePyramid( frame->image(), &pyramid );
        frame->setImagePyramid( pyramid );
    }

}

void CPUFlowTracker::extract( ProcFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    processor()->extractPoints( frame->image(), frame->mask(), &cornerPoints, frame->extractionCornersCount() );

    frame->addCornerPoints( cornerPoints );
}

void CPUFlowTracker::prepareFrame( ProcStereoFrame *frame )
{
    prepareFrame( frame->leftFrame().get() );
    prepareFrame( frame->rightFrame().get() );
}

void CPUFlowTracker::prepareFrame( ConsecutiveFrames *frame )
{
    prepareFrame( frame->frame2().get() );
}

void CPUFlowTracker::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
}

void CPUFlowTracker::extract( ConsecutiveFrames * )
{
}

void CPUFlowTracker::match( ProcStereoFrame *frame )
{
    std::vector< FlowTrackResult > trackResults;

    processor()->track( frame->leftFrame()->imagePyramid(), frame->leftFrame()->cornerPoints(), frame->rightFrame()->imagePyramid(), &trackResults );

    for ( auto &i : trackResults ) {
        auto rightIndex = frame->rightFrame()->addCornerPoint( i );
        frame->createFlowPoint( i.index, rightIndex );

    }

}

void CPUFlowTracker::match( ConsecutiveFrames *frame )
{
    auto frame1 = frame->frame1();
    auto frame2 = frame->frame2();

    std::vector< FlowTrackResult > trackResults;

    processor()->track( frame1->imagePyramid(), frame1->cornerPoints(), frame2->imagePyramid(), &trackResults );

    for ( auto &i : trackResults ) {
        auto flowIndex = frame2->addCornerPoint( i );
        auto flowPoint = frame2->createFlowPoint( flowIndex );

        auto prevFlowPoint = frame1->flowPoint( i.index );

        if ( prevFlowPoint ) {

            auto track = prevFlowPoint->parentTrack();

            if ( !track ) {
                track = frame1->createTrack();
                track->addPoint( prevFlowPoint );
            }

            track->addPoint( flowPoint );

        }

    }

}

CPUFlowProcessor *CPUFlowTracker::processor() const
{
    return dynamic_cast< CPUFlowProcessor* >( _pointsProcessor.get() );
}

// GPUFlowTracker
GPUFlowTracker::GPUFlowTracker()
{
    initialize();
}

void GPUFlowTracker::initialize()
{
    _pointsProcessor = std::unique_ptr< FlowProcessor >( new GPUFlowProcessor() );
}

void GPUFlowTracker::prepareFrame( ProcFrame * )
{
}

void GPUFlowTracker::extract( ProcFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    processor()->extractPoints( frame->image(), frame->mask(), &cornerPoints, frame->extractionCornersCount() );

    frame->addCornerPoints( cornerPoints );
}

void GPUFlowTracker::prepareFrame( ProcStereoFrame *frame )
{
    prepareFrame( frame->leftFrame().get() );
    prepareFrame( frame->rightFrame().get() );
}

void GPUFlowTracker::prepareFrame( ConsecutiveFrames *frame )
{
    prepareFrame( frame->frame2().get() );
}

void GPUFlowTracker::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
}

void GPUFlowTracker::extract( ConsecutiveFrames * )
{
}

void GPUFlowTracker::match( ProcStereoFrame *frame )
{
    std::vector< FlowTrackResult > trackResults;

    processor()->track( frame->leftFrame()->image(), frame->leftFrame()->cornerPoints(), frame->rightFrame()->image(), &trackResults );

    for ( auto &i : trackResults ) {
        auto rightIndex = frame->rightFrame()->addCornerPoint( i );
        frame->createFlowPoint( i.index, rightIndex );

    }

}

void GPUFlowTracker::match( ConsecutiveFrames *frame )
{
}

GPUFlowProcessor *GPUFlowTracker::processor() const
{
    return dynamic_cast< GPUFlowProcessor* >( _pointsProcessor.get() );
}

// FeatureTracker

// SiftTracker
SiftTracker::SiftTracker()
{
    initialize();
}

void SiftTracker::initialize()
{
    _descriptorProcessor = std::unique_ptr< FullProcessor >( new SiftProcessor() );
    _featuresMatcher = std::unique_ptr< DescriptorMatcher >( new FlannMatcher() );
}

void SiftTracker::prepareFrame( ProcFrame * )
{
}

void SiftTracker::extract( ProcFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    std::vector< cv::KeyPoint > keypoints;
    cv::Mat descriptors;

    processor()->extractAndCompute( frame->image(), frame->mask(), &keypoints, &descriptors );

    frame->setKeyPoints( keypoints );
    frame->setDescriptors( descriptors );
}

void SiftTracker::prepareFrame( ProcStereoFrame *frame )
{
    prepareFrame( frame->leftFrame().get() );
    prepareFrame( frame->rightFrame().get() );
}

void SiftTracker::prepareFrame( ConsecutiveFrames *frame )
{
    prepareFrame( frame->frame2().get() );
}

void SiftTracker::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
    extract( frame->rightFrame().get() );
}

void SiftTracker::extract( ConsecutiveFrames *frame )
{
    extract( frame->frame2().get() );
}

void SiftTracker::match( ProcStereoFrame *frame )
{
    std::vector< cv::DMatch > matches;

    matcher()->match( frame->leftFrame()->keyPoints(), frame->leftFrame()->descriptors(), frame->rightFrame()->keyPoints(), frame->rightFrame()->descriptors(), &matches );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );

}

void SiftTracker::match( ConsecutiveFrames *frame )
{
}

SiftProcessor *SiftTracker::processor() const
{
    return dynamic_cast< SiftProcessor* >( _descriptorProcessor.get() );
}

FlannMatcher *SiftTracker::matcher() const
{
    return dynamic_cast< FlannMatcher* >( _featuresMatcher.get() );
}

// OrbTracker
OrbTracker::OrbTracker()
{
    initialize();
}

void OrbTracker::initialize()
{
    _descriptorProcessor = std::unique_ptr< FullProcessor >( new OrbProcessor() );
    _featuresMatcher = std::unique_ptr< DescriptorMatcher >( new BFMatcher() );
}

void OrbTracker::prepareFrame( ProcFrame * )
{
}

void OrbTracker::extract( ProcFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    std::vector< cv::KeyPoint > keypoints;
    cv::Mat descriptors;

    processor()->extractAndCompute( frame->image(), frame->mask(), &keypoints, &descriptors );

    frame->setKeyPoints( keypoints );
    frame->setDescriptors( descriptors );
}

void OrbTracker::prepareFrame( ProcStereoFrame *frame )
{
    prepareFrame( frame->leftFrame().get() );
    prepareFrame( frame->rightFrame().get() );
}

void OrbTracker::prepareFrame( ConsecutiveFrames *frame )
{
    prepareFrame( frame->frame2().get() );
}

void OrbTracker::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
    extract( frame->rightFrame().get() );
}

void OrbTracker::extract( ConsecutiveFrames *frame )
{
    extract( frame->frame2().get() );
}

void OrbTracker::match( ProcStereoFrame *frame )
{
    std::vector< cv::DMatch > matches;

    matcher()->match( frame->leftFrame()->keyPoints(), frame->leftFrame()->descriptors(), frame->rightFrame()->keyPoints(), frame->rightFrame()->descriptors(), &matches );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );

}

void OrbTracker::match( ConsecutiveFrames *frame )
{
}

OrbProcessor *OrbTracker::processor() const
{
    return dynamic_cast< OrbProcessor* >( _descriptorProcessor.get() );
}

BFMatcher *OrbTracker::matcher() const
{
    return dynamic_cast< BFMatcher* >( _featuresMatcher.get() );
}

// AKazeTracker
AKazeTracker::AKazeTracker()
{
    initialize();
}

void AKazeTracker::initialize()
{
    _descriptorProcessor = std::make_unique< AKazeProcessor >();
    _featuresMatcher = std::make_unique< BFMatcher >();
}

void AKazeTracker::prepareFrame( ProcFrame * )
{

}

void AKazeTracker::extract( ProcFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    std::vector< cv::KeyPoint > keypoints;
    cv::Mat descriptors;

    processor()->extractAndCompute( frame->image(), frame->mask(), &keypoints, &descriptors );

    frame->setKeyPoints( keypoints );
    frame->setDescriptors( descriptors );
}

void AKazeTracker::prepareFrame( ProcStereoFrame *frame )
{
    prepareFrame( frame->leftFrame().get() );
    prepareFrame( frame->rightFrame().get() );
}

void AKazeTracker::prepareFrame( ConsecutiveFrames *frame )
{
    prepareFrame( frame->frame2().get() );
}

void AKazeTracker::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
    extract( frame->rightFrame().get() );
}

void AKazeTracker::extract( ConsecutiveFrames *frame )
{
    extract( frame->frame2().get() );
}

void AKazeTracker::match( ProcStereoFrame *frame )
{
    std::vector< cv::DMatch > matches;

    matcher()->match( frame->leftFrame()->keyPoints(), frame->leftFrame()->descriptors(), frame->rightFrame()->keyPoints(), frame->rightFrame()->descriptors(), &matches );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );

}

void AKazeTracker::match( ConsecutiveFrames *frame )
{
}

AKazeProcessor *AKazeTracker::processor() const
{
    return dynamic_cast< AKazeProcessor* >( _descriptorProcessor.get() );
}

BFMatcher *AKazeTracker::matcher() const
{
    return dynamic_cast< BFMatcher* >( _featuresMatcher.get() );
}

// SuperGlueTracker
SuperGlueTracker::SuperGlueTracker()
{
    initialize();
}

void SuperGlueTracker::initialize()
{
    _processor = std::make_unique< SuperGlueProcessor >( "superpoint_fp32_1024.eng", "superglue_fp32.eng" );
}

void SuperGlueTracker::prepareFrame( ProcFrame * )
{
}

void SuperGlueTracker::extract( ProcFrame *frame )
{
    if ( frame->keyPoints().empty() ) {
        std::vector< cv::KeyPoint > keypoints;
        cv::Mat descriptors;

        CvImage gray;

        cv::cvtColor( frame->image(), gray, cv::COLOR_BGR2GRAY );

        _processor->extract( gray, frame->mask(), &keypoints, &descriptors );

        frame->setKeyPoints( keypoints );
        frame->setDescriptors( descriptors );

    }

}

void SuperGlueTracker::prepareFrame( ProcStereoFrame *frame )
{
    prepareFrame( frame->leftFrame().get() );
    prepareFrame( frame->rightFrame().get() );
}

void SuperGlueTracker::prepareFrame( ConsecutiveFrames *frame )
{
    prepareFrame( frame->frame2().get() );
}

void SuperGlueTracker::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
    extract( frame->rightFrame().get() );
}

void SuperGlueTracker::extract( ConsecutiveFrames *frame )
{
    extract( frame->frame2().get() );
}

void SuperGlueTracker::match( ProcStereoFrame *frame )
{
    std::vector< cv::DMatch > matches;

    _processor->match( frame->leftFrame()->image().size(), frame->rightFrame()->image().size(), frame->leftFrame()->keyPoints(), frame->rightFrame()->keyPoints(), frame->leftFrame()->descriptors(), frame->rightFrame()->descriptors(), &matches );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );
}

void SuperGlueTracker::match( ConsecutiveFrames *frame )
{
    auto frame1 = frame->frame1();
    auto frame2 = frame->frame2();

    std::vector< cv::DMatch > matches;

    processor()->match( frame1->image().size(), frame2->image().size(), frame1->keyPoints(), frame2->keyPoints(), frame1->descriptors(), frame2->descriptors(), &matches );

    for ( auto &i : matches ) {
        auto featurePoint = frame2->createFeaturePoint( i.trainIdx );

        auto prevFeaturePoint = frame1->featurePoint( i.queryIdx );

        if ( prevFeaturePoint ) {

            auto track = prevFeaturePoint->parentTrack();

            if ( !track ) {
                track = frame1->createTrack();
                track->addPoint( prevFeaturePoint );
            }

            track->addPoint( featurePoint );

        }

    }
}

SuperGlueProcessor *SuperGlueTracker::processor() const
{
    return _processor.get();
}


}
