#include "src/common/precompiled.h"

#include "tracker.h"

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

std::vector< cv::Mat > CPUFlowTracker::buildPyramid( const CvImage &image )
{
    std::vector< cv::Mat > ret;

    processor()->buildImagePyramid( image, &ret );

    return ret;
}

void CPUFlowTracker::extractCorners( const CvImage &image, const cv::Mat &mask, const size_t count, std::vector< cv::Point2f > *cornerPoints )
{
    processor()->extractPoints( image, mask, cornerPoints, count );
}

void CPUFlowTracker::match( const std::vector< cv::Mat > &pyr1, const std::vector< cv::Mat > &pyr2 , const std::vector< cv::Point2f > &points, std::vector< FlowTrackResult > *results )
{
    processor()->track( pyr1, points, pyr2, results );
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
}

}
