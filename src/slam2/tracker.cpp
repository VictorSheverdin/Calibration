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

void CPUFlowTracker::match( ConsecutiveStereoFrames *frame )
{

}

CPUFlowProcessor *CPUFlowTracker::processor() const
{
    return dynamic_cast< CPUFlowProcessor* >( _pointsProcessor.get() );
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
}

void SiftTracker::match( ProcStereoFrame *frame )
{
    std::vector< cv::DMatch > matches;

    matcher()->match( frame->leftFrame()->featurePoints(), frame->leftFrame()->descriptors(), frame->rightFrame()->featurePoints(), frame->rightFrame()->descriptors(), &matches );

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

}
