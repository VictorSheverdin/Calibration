#include "src/common/precompiled.h"

#include "tracker.h"

#include "frame.h"

namespace slam2 {

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

void CPUFlowTracker::prepareFrame( ProcStereoFrame *frame )
{
    std::vector< cv::Mat > leftPyramid, rightPyramid;

    processor()->buildImagePyramid( frame->leftFrame()->image(), &leftPyramid );
    processor()->buildImagePyramid( frame->rightFrame()->image(), &rightPyramid );

    frame->setImagePyramid( leftPyramid, rightPyramid );
}

void CPUFlowTracker::extractFeatures( ProcStereoFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    processor()->extractPoints( frame->leftFrame()->image(), frame->leftFrame()->mask(), &cornerPoints, frame->leftFrame()->extractionCornersCount() );

    frame->leftFrame()->addCornerPoints( cornerPoints );

    std::vector< FlowTrackResult > trackResults;

    processor()->track( frame->leftFrame()->imagePyramid(), cornerPoints, frame->rightFrame()->imagePyramid(), &trackResults );

    for ( auto &i : trackResults ) {
        auto rightIndex = frame->rightFrame()->addCornerPoint( i );
        frame->createFlowPoint( i.index, rightIndex );

    }

}

void CPUFlowTracker::match( ConsecutiveStereoFrames *frame )
{
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

void GPUFlowTracker::prepareFrame( ProcStereoFrame *frame )
{
}

void GPUFlowTracker::extractFeatures( ProcStereoFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    processor()->extractPoints( frame->leftFrame()->image(), frame->leftFrame()->mask(), &cornerPoints, frame->leftFrame()->extractionCornersCount() );

    frame->leftFrame()->addCornerPoints( cornerPoints );

    std::vector< FlowTrackResult > trackResults;

    processor()->track( frame->leftFrame()->image(), cornerPoints, frame->rightFrame()->image(), &trackResults );

    for ( auto &i : trackResults ) {
        auto rightIndex = frame->rightFrame()->addCornerPoint( i );
        frame->createFlowPoint( i.index, rightIndex );

    }

}

void GPUFlowTracker::match( ConsecutiveStereoFrames *frame )
{
}

GPUFlowProcessor *GPUFlowTracker::processor() const
{
    return dynamic_cast< GPUFlowProcessor* >( _pointsProcessor.get() );
}

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

void SiftTracker::prepareFrame( ProcStereoFrame * )
{
}

void SiftTracker::extractFeatures( ProcStereoFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    std::vector< cv::KeyPoint > leftKeypoints, rightKeypoints;
    cv::Mat leftDescriptors, rightDescriptors;

    processor()->extractAndCompute( frame->leftFrame()->image(), frame->leftFrame()->mask(), &leftKeypoints, &leftDescriptors );
    processor()->extractAndCompute( frame->rightFrame()->image(), frame->rightFrame()->mask(), &rightKeypoints, &rightDescriptors );

    frame->setFeaturePoints( leftKeypoints, rightKeypoints );
    frame->setDescriptors( leftDescriptors, rightDescriptors );

    std::vector< cv::DMatch > matches;

    matcher()->match( leftKeypoints, leftDescriptors, rightKeypoints, rightDescriptors, &matches );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );

}

void SiftTracker::match( ConsecutiveStereoFrames *frame )
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

void OrbTracker::prepareFrame( ProcStereoFrame * )
{
}

void OrbTracker::extractFeatures( ProcStereoFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    std::vector< cv::KeyPoint > leftKeypoints, rightKeypoints;
    cv::Mat leftDescriptors, rightDescriptors;

    processor()->extractAndCompute( frame->leftFrame()->image(), frame->leftFrame()->mask(), &leftKeypoints, &leftDescriptors );
    processor()->extractAndCompute( frame->rightFrame()->image(), frame->rightFrame()->mask(), &rightKeypoints, &rightDescriptors );

    frame->setFeaturePoints( leftKeypoints, rightKeypoints );
    frame->setDescriptors( leftDescriptors, rightDescriptors );

    std::vector< cv::DMatch > matches;

    matcher()->match( leftKeypoints, leftDescriptors, rightKeypoints, rightDescriptors, &matches );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );
}

void OrbTracker::match( ConsecutiveStereoFrames *frame )
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

void AKazeTracker::prepareFrame( ProcStereoFrame * )
{
}

void AKazeTracker::extractFeatures( ProcStereoFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    std::vector< cv::KeyPoint > leftKeypoints, rightKeypoints;
    cv::Mat leftDescriptors, rightDescriptors;

    processor()->extractAndCompute( frame->leftFrame()->image(), frame->leftFrame()->mask(), &leftKeypoints, &leftDescriptors );
    processor()->extractAndCompute( frame->rightFrame()->image(), frame->rightFrame()->mask(), &rightKeypoints, &rightDescriptors );

    frame->setFeaturePoints( leftKeypoints, rightKeypoints );
    frame->setDescriptors( leftDescriptors, rightDescriptors );

    std::vector< cv::DMatch > matches;

    matcher()->match( leftKeypoints, leftDescriptors, rightKeypoints, rightDescriptors, &matches );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );

}

void AKazeTracker::match( ConsecutiveStereoFrames *frame )
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

void SuperGlueTracker::prepareFrame( ProcStereoFrame * )
{
}

void SuperGlueTracker::extractFeatures( ProcStereoFrame *frame )
{
    std::vector< cv::KeyPoint > leftKeypoints, rightKeypoints;
    std::vector< cv::DMatch > matches;

    CvImage leftGray, rightGray;

    cv::cvtColor( frame->leftFrame()->image(), leftGray, cv::COLOR_BGR2GRAY );
    cv::cvtColor( frame->rightFrame()->image(), rightGray, cv::COLOR_BGR2GRAY );

    _processor->extractAndMatch( leftGray, frame->leftFrame()->mask(), rightGray, frame->rightFrame()->mask(), &leftKeypoints, &rightKeypoints, 1024, &matches );

    frame->setFeaturePoints( leftKeypoints, rightKeypoints );

    for ( auto &i : matches )
        frame->createFeaturePoint( i.queryIdx, i.trainIdx );
}

void SuperGlueTracker::match( ConsecutiveStereoFrames *frame )
{

}

SuperGlueProcessor *SuperGlueTracker::processor() const
{
    return _processor.get();
}


}
