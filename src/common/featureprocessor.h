#pragma once

#include "src/common/image.h"

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/cudafeatures2d.hpp>

#include <opencv2/optflow.hpp>
#include <opencv2/cudaoptflow.hpp>

struct FlowTrackResult : public cv::Point2f
{
public:
    FlowTrackResult( size_t index, cv::Point2f point, float error, float miss );

    size_t index;
    float error;
    float miss;

    bool operator<( const FlowTrackResult &other ) const;

};

class FeatureProcessorBase
{
protected:
    FeatureProcessorBase() = default;
    virtual ~FeatureProcessorBase() = default;
};

class FlowProcessor : public FeatureProcessorBase
{
public:
    void extractPoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::Point2f > *points, const size_t count );

    double extractPrecision() const;
    void setExtractPrecision( const double value );

    double checkDistance() const;
    void setCheckDistance( const double value );

    void setExtractionDistance( const double value );
    double extractionDistance() const;

    virtual size_t winSize() const = 0;
    virtual void setWinSize( const size_t value ) = 0;

    virtual size_t levels() const = 0;
    virtual void setLevels( const size_t value ) = 0;

    double ransacReprojectionThreshold() const;
    void setRansacReprojectionThreshold( const double &value );

    double ransacConfidence() const;
    void setRansacConfidence( const double &value );

    cv::Mat epiTest( const std::vector< cv::Point2f > &sourcePoints, const std::vector< cv::Point2f > &targetPoints, std::vector<int> *inliers );

protected:
    FlowProcessor();

    double m_checkDistance;

    double m_extractPrecision;
    double m_extractDistance;
    double m_blockSize;

    double m_ransacReprojectionThreshold;
    double m_ransacConfidence;

private:
    void initialize();
};

class GPUFlowProcessor : public FlowProcessor
{
public:
    GPUFlowProcessor();

    virtual size_t winSize() const override;
    virtual void setWinSize( const size_t value ) override;

    virtual size_t levels() const override;
    virtual void setLevels( const size_t value ) override;

    void track( const CvImage &sourceImage, const std::vector< cv::Point2f > &sourcePoints, const CvImage &targetImage, std::vector< FlowTrackResult > *trackedPoints );

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

    void track( const std::vector< cv::Mat > &sourceImagePyramid, const std::vector< cv::Point2f > &sourcePoints, const std::vector< cv::Mat > &targetImagePyramid, std::vector< FlowTrackResult > *trackedPoints );

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

class FeatureProcessor : public FeatureProcessorBase
{
public:

protected:
    FeatureProcessor() = default;

    cv::Ptr< cv::Feature2D > m_processor;

};

class KeyPointProcessor : public virtual FeatureProcessor
{
public:
    void extractKeypoints( const CvImage &image, const cv::Mat &mask, std::vector< cv::KeyPoint > *keypoints );

protected:
    KeyPointProcessor() = default;

};

class DescriptorProcessor : public virtual FeatureProcessor
{
public:
    void extractDescriptors( const CvImage &image, std::vector< cv::KeyPoint > &keypoints, cv::Mat *descriptors );

protected:
    DescriptorProcessor() = default;

};

class FullProcessor : public KeyPointProcessor, public DescriptorProcessor
{
public:
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

    void setQualityLevel( double value );
    double qualityLevel() const;

    void setMinDistance( double value );
    double minDistance() const;

    void setBlockSize( int value );
    int blockSize() const;

    void setHarrisDetector( bool value );
    bool harrisDetector() const;

    void setK(double value );
    double k() const;


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

    void setNonmaxSuppression( bool value );
    bool nonmaxSuppression();

    void setType( cv::FastFeatureDetector::DetectorType value );
    cv::FastFeatureDetector::DetectorType type();

private:
    void initialize();

};

namespace marker {
    class SuperPointDetector;
    class KeypointSelector;
    class SuperGlueMatcher;
}

class SuperGlueProcessor : public FeatureProcessorBase
{
public:
    SuperGlueProcessor( const std::string &detectorModelFile, const std::string &matcherModelFile );

    void setMatchingThreshold( const double value );
    int matchingThreshold() const;

    void extractAndMatch( const CvImage &image1, const cv::Mat &mask1, const CvImage &image2, const cv::Mat &mask2,
                           std::vector< cv::KeyPoint > *keypoints1, std::vector< cv::KeyPoint > *keypoints2, const size_t count, std::vector< cv::DMatch > *matches );

    void match( const CvImage &image1, const CvImage &image2, const std::vector< cv::KeyPoint > &keypoints1, const std::vector< cv::KeyPoint > &keypoints2, std::vector< cv::DMatch > *matches );

protected:
    std::shared_ptr< marker::SuperPointDetector > _detector;
    std::shared_ptr< marker::KeypointSelector > _keypointSelector;
    std::shared_ptr< marker::SuperGlueMatcher > _matcher;

    double _matcherThreshold;

    static const size_t _maxPointsCount = 1024;
private:
    void initialize( const std::string &detectorModelFile, const std::string &matcherModelFile );

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

class DescriptorMatcher : public FeatureMatcherBase
{
public:
    DescriptorMatcher();

    cv::Mat match( const std::vector<cv::KeyPoint> &queryKeypoints, const cv::Mat &queryDescriptors,
                const std::vector<cv::KeyPoint> &trainKeypoints, const cv::Mat &trainDescriptors,
                std::vector< cv::DMatch > *matches );

protected:
    static double m_threshold;

    cv::Ptr< cv::DescriptorMatcher > m_matcher;

private:
};

class BFMatcher : public DescriptorMatcher
{
public:
    BFMatcher();

private:
    void initialize();

};

class FlannMatcher : public DescriptorMatcher
{
public:
    FlannMatcher();

private:
    void initialize();

};

