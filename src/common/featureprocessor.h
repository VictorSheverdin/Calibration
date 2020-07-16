#pragma once

#include "src/common/image.h"

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/cudafeatures2d.hpp>

#include <opencv2/optflow.hpp>
#include <opencv2/cudaoptflow.hpp>

class FlowProcessor
{
public:
    void extractPoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::Point2f > *points );

    size_t count() const;
    void setCount( const size_t value );

    double minDistance() const;
    void setMinDistance( const double value );

    double extractPrecision() const;
    void setExtractPrecision( const double value );

    virtual size_t winSize() const = 0;
    virtual void setWinSize( const size_t value ) = 0;

    virtual size_t levels() const = 0;
    virtual void setLevels( const size_t value ) = 0;

    double ransacReprojectionThreshold() const;
    void setRansacReprojectionThreshold( const double &value );

    double ransacConfidence() const;
    void setRansacConfidence( const double &value );

protected:
    FlowProcessor();

    static const double m_checkDistance;
    static const double m_errorRatio;

    size_t m_count;
    double m_minDistance;
    double m_extractPrecision;

    double m_ransacReprojectionThreshold;
    double m_ransacConfidence;

private:
    void initialize();
};

struct PointTrackResult {

public:
    PointTrackResult( size_t index, cv::Point2f point, float error );

    size_t index;
    cv::Point2f point;
    float error;

    bool operator<( const PointTrackResult &other ) const;

};

class GPUFlowProcessor : public FlowProcessor
{
public:
    GPUFlowProcessor();

    virtual size_t winSize() const override;
    virtual void setWinSize( const size_t value ) override;

    virtual size_t levels() const override;
    virtual void setLevels( const size_t value ) override;

    cv::Mat track( const CvImage &sourceImage, const std::vector< cv::Point2f > &sourcePoints, const CvImage &targetImage, std::set< PointTrackResult > *trackedPoints );

protected:
    cv::Ptr< cv::cuda::SparsePyrLKOpticalFlow > m_opticalProcessor;

    cv::cuda::GpuMat m_gpuSourceImage;
    cv::cuda::GpuMat m_gpuTargetImage;
    cv::cuda::GpuMat m_gpuSourcePoints;
    cv::cuda::GpuMat m_gpuOpticalPoints;
    cv::cuda::GpuMat m_gpuCheckPoints;
    cv::cuda::GpuMat m_gpuStatuses;
    cv::cuda::GpuMat m_gpuCheckStatuses;
    cv::cuda::GpuMat m_gpuErr;
    cv::cuda::GpuMat m_gpuCheckErr;

private:
    void initialize();

};

class CPUFlowProcessor : public FlowProcessor
{
public:
    CPUFlowProcessor();

    cv::Mat track( const std::vector< cv::Mat > &sourceImagePyramid, const std::vector< cv::Point2f > &sourcePoints, const std::vector< cv::Mat > &targetImagePyramid, std::set< PointTrackResult > *trackedPoints );

    void buildImagePyramid( const CvImage &image, std::vector< cv::Mat > *imagePyramid );

    virtual size_t winSize() const override;
    virtual void setWinSize( const size_t value ) override;

    virtual size_t levels() const override;
    virtual void setLevels( const size_t value ) override;

    void setPrecission( const double value );
    double precission() const;

    void setMinEigenValue( const double value );
    double minEigenValue() const;

protected:
    size_t m_winSize;
    size_t m_levels;

    cv::TermCriteria m_termCriteria;

    double m_minEigenValue;

private:
    void initialize();

};

class FeatureProcessorBase
{
public:
    virtual ~FeatureProcessorBase() = default;

protected:
    FeatureProcessorBase() = default;

    cv::Ptr< cv::Feature2D > m_processor;

};

class KeyPointProcessor : public FeatureProcessorBase
{
public:
    void extractKeypoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints );

protected:
    KeyPointProcessor() = default;

};

class DescriptorProcessor : public FeatureProcessorBase
{
public:
    void extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors );

protected:
    DescriptorProcessor() = default;

};

class FullProcessor : public FeatureProcessorBase
{
public:
    void extractKeypoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints );
    void extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors );

    void extractAndCompute( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints, cv::Mat *descriptors );

protected:
    FullProcessor() = default;

};

class GFTTProcessor : public KeyPointProcessor
{
public:
    GFTTProcessor();

    cv::Ptr< cv::GFTTDetector > processor() const;

    void setMaxFeatures( const int value );
    int maxFeatures() const;

private:
    void initialize();

};

class FastProcessor : public KeyPointProcessor
{
public:
    FastProcessor();

    cv::Ptr< cv::FastFeatureDetector > processor() const;

    void setThreshold( const int value );
    int threshold() const;

private:
    void initialize();

};

class DaisyProcessor : public DescriptorProcessor
{
public:
    DaisyProcessor();

private:
    void initialize();

};

class FreakProcessor : public DescriptorProcessor
{
public:
    FreakProcessor();

private:
    void initialize();

};

class SiftProcessor : public FullProcessor
{
public:
    SiftProcessor();

private:
    void initialize();

};

class SurfProcessor : public FullProcessor
{
public:
    SurfProcessor();

private:
    void initialize();

};

class OrbProcessor : public FullProcessor
{
public:
    OrbProcessor();

private:
    void initialize();

};

class KazeProcessor : public FullProcessor
{
public:
    KazeProcessor();

private:
    void initialize();

};

class AKazeProcessor : public FullProcessor
{
public:
    AKazeProcessor();

private:
    void initialize();

};

class FeatureMatcherBase
{
public:
    FeatureMatcherBase();
    virtual ~FeatureMatcherBase() = default;

protected:
private:
};

class DescriptorMatcherBase : public FeatureMatcherBase
{
public:
    DescriptorMatcherBase();

    cv::Mat match( const std::vector<cv::KeyPoint> &queryKeypoints, const cv::Mat &queryDescriptors,
                const std::vector<cv::KeyPoint> &trainKeypoints, const cv::Mat &trainDescriptors,
                std::vector< cv::DMatch > *matches );

protected:
    static double m_threshold;

    cv::Ptr< cv::DescriptorMatcher > m_matcher;

private:
};

class BFMatcher : public DescriptorMatcherBase
{
public:
    BFMatcher();

private:
    void initialize();

};

class FlannMatcher : public DescriptorMatcherBase
{
public:
    FlannMatcher();

private:
    void initialize();

};

