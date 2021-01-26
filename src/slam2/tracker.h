#pragma once

#include "src/common/featureprocessor.h"

#include "alias.h"

namespace slam2 {

class FlowTracker
{
public:
    virtual std::vector< cv::Mat > buildPyramid( const CvImage &image ) = 0;
    virtual void extractCorners( const CvImage &image, const cv::Mat &mask, const size_t count, std::vector< cv::Point2f > *cornerPoints ) = 0;
    virtual void match( const std::vector< cv::Mat > &pyr1, const std::vector< cv::Mat > &pyr2, const std::vector< cv::Point2f > &points, std::vector< FlowTrackResult > *results ) = 0;

    double extractPrecision() const;
    void setExtractPrecision( const double value );

    void setExtractionDistance( const double value );
    double extractionDistance() const;

    size_t winSize() const;
    void setWinSize( const size_t value );

    size_t levels() const;
    void setLevels( const size_t value );

protected:
    FlowTracker() = default;

    std::unique_ptr< FlowProcessor > _pointsProcessor;

};

class CPUFlowTracker : public FlowTracker
{
public:
    CPUFlowTracker();

    std::vector< cv::Mat > buildPyramid( const CvImage &image ) override;
    void extractCorners( const CvImage &image, const cv::Mat &mask, const size_t count, std::vector< cv::Point2f > *cornerPoints ) override;
    void match( const std::vector< cv::Mat > &pyr1, const std::vector< cv::Mat > &pyr2, const std::vector< cv::Point2f > &points, std::vector< FlowTrackResult > *results ) override;

protected:
    CPUFlowProcessor *processor() const;

private:
    void initialize();

};

class FeatureTracker
{
public:

protected:
    FeatureTracker() = default;

    std::unique_ptr< FullProcessor > _descriptorProcessor;

};

class SiftTracker : public FeatureTracker
{
public:
    SiftTracker();

protected:
    FlannMatcher _featuresMatcher;

private:
    void initialize();

};

}
