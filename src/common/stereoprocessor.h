#pragma once

#include "calibrationdatabase.h"

#include "image.h"
#include "colorpoint.h"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rectificationprocessor.h"

class DisparityProcessorBase
{
public:
    DisparityProcessorBase();

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) = 0;

protected:
    CvImage preprocess( const CvImage img );
};

class BMDisparityProcessor : public DisparityProcessorBase
{
public:
    BMDisparityProcessor();

    int getMinDisparity() const;
    void setMinDisparity( const int minDisparity );

    int getNumDisparities() const;
    void setNumDisparities( const int numDisparities );

    int getBlockSize() const;
    void setBlockSize( const int blockSize );

    int getSpeckleWindowSize() const;
    void setSpeckleWindowSize( const int speckleWindowSize );

    int getSpeckleRange() const;
    void setSpeckleRange( const int speckleRange );

    int getDisp12MaxDiff() const;
    void setDisp12MaxDiff( const int disp12MaxDiff );

    int getPreFilterType() const;
    void setPreFilterType( const int preFilterType );

    int getPreFilterSize() const;
    void setPreFilterSize( const int preFilterSize );

    int getPreFilterCap() const;
    void setPreFilterCap( const int preFilterCap );

    int getTextureThreshold() const;
    void setTextureThreshold( const int textureThreshold );

    int getUniquenessRatio() const;
    void setUniquenessRatio( const int uniquenessRatio );

    cv::Rect getROI1() const;
    void setROI1( const cv::Rect &roi1 );

    cv::Rect getROI2() const;
    void setROI2( const cv::Rect &roi2 );

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) override;

protected:
//    cv::Ptr< cv::ximgproc::DisparityWLSFilter > m_wlsFilter;
    cv::Ptr< cv::StereoBM > m_leftMatcher;
//    cv::Ptr< cv::StereoMatcher > m_rightMatcher;

private:
    void initialize();

};

class BMGPUDisparityProcessor : public DisparityProcessorBase
{
public:
    BMGPUDisparityProcessor();

    int getNumDisparities() const;
    void setNumDisparities( const int numDisparities );

    int getBlockSize() const;
    void setBlockSize( const int blockSize );

    int getPreFilterType() const;
    void setPreFilterType( const int preFilterType );

    int getPreFilterCap() const;
    void setPreFilterCap( const int preFilterCap );

    int getTextureThreshold() const;
    void setTextureThreshold( const int textureThreshold );

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) override;

protected:
    cv::Ptr< cv::cuda::StereoBM > m_matcher;

private:
    void initialize();

};

class GMDisparityProcessor : public DisparityProcessorBase
{
public:
    GMDisparityProcessor();

    int getMode() const;
    void setMode( int mode );

    int getMinDisparity() const;
    void setMinDisparity( const int minDisparity );

    int getNumDisparities() const;
    void setNumDisparities( const int numDisparities );

    int getBlockSize() const;
    void setBlockSize( const int blockSize );

    int getSpeckleWindowSize() const;
    void setSpeckleWindowSize( const int speckleWindowSize );

    int getSpeckleRange() const;
    void setSpeckleRange( const int speckleRange );

    int getDisp12MaxDiff() const;
    void setDisp12MaxDiff( const int disp12MaxDiff );

    int getPreFilterCap() const;
    void setPreFilterCap( const int preFilterCap );

    int getUniquenessRatio() const;
    void setUniquenessRatio( const int uniquenessRatio );

    int getP1() const;
    void setP1( int p1 );

    int getP2() const;
    void setP2( int p2 );

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) override;

protected:
    cv::Ptr< cv::StereoSGBM > m_matcher;

private:
    void initialize();

};

class BPDisparityProcessor : public DisparityProcessorBase
{
public:
    BPDisparityProcessor();

    int getNumDisparities() const;
    void setNumDisparities( const int value );

    int getNumIterations() const;
    void setNumIterations( const int value );

    int getNumLevels() const;
    void setNumLevels( const int value );

    double getMaxDataTerm() const;
    void setMaxDataTerm( const double value );

    double getDataWeight() const;
    void setDataWeight( const double value );

    double getMaxDiscTerm() const;
    void setMaxDiscTerm( const double value );

    double getDiscSingleJump() const;
    void setDiscSingleJump( const double value );

    int getMsgType() const;
    void setMsgType( const int value );

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) override;

protected:
    cv::Ptr< cv::cuda::StereoBeliefPropagation > m_matcher;

private:
    void initialize();

};

class CSBPDisparityProcessor : public DisparityProcessorBase
{
public:
    CSBPDisparityProcessor();

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) override;

protected:
    cv::Ptr< cv::cuda::StereoConstantSpaceBP > m_matcher;

private:
    void initialize();

};

class StereoProcessorBase
{
public:
    StereoProcessorBase();
    StereoProcessorBase( const std::shared_ptr< DisparityProcessorBase > &proc );

    const std::shared_ptr< DisparityProcessorBase > &disparityProcessor() const;
    void setDisparityProcessor( const std::shared_ptr< DisparityProcessorBase > &proc );

    cv::Mat processDisparity( const CvImage &left, const CvImage &right );

protected:
    std::shared_ptr< DisparityProcessorBase > m_disparityProcessor;

};

class StereoProcessor : public StereoProcessorBase
{
public:
    StereoProcessor();
    StereoProcessor( const std::shared_ptr< DisparityProcessorBase > &proc );

    void setDisparityToDepthMatrix( const cv::Mat & mat );
    const cv::Mat &disparityToDepthMatrix() const;

    pcl::PointCloud< pcl::PointXYZRGB >::Ptr processPointCloud( const CvImage &left, const CvImage &right );
    std::list< ColorPoint3d > processPointList( const CvImage &left, const CvImage &right );

protected:
    cv::Mat m_disparityToDepthMatrix;

    cv::Mat reprojectPoints( const cv::Mat &disparity );
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr producePointCloud( const cv::Mat &points, const CvImage &leftImage );
    std::list< ColorPoint3d > producePointList( const cv::Mat &points, const CvImage &leftImage );

};

class TriangulationProcessor
{

};

class BMStereoProcessor : public StereoProcessor
{
public:
    BMStereoProcessor();

    const std::shared_ptr< BMDisparityProcessor > disparityProcessor() const;

    int getMinDisparity() const;
    void setMinDisparity( const int minDisparity );

    int getNumDisparities() const;
    void setNumDisparities( const int numDisparities );

    int getBlockSize() const;
    void setBlockSize( const int blockSize );

    int getTextureThreshold() const;
    void setTextureThreshold( const int textureThreshold );

    int getSpeckleWindowSize() const;
    void setSpeckleWindowSize( const int speckleWindowSize );

    int getSpeckleRange() const;
    void setSpeckleRange( const int speckleRange );

    int getDisp12MaxDiff() const;
    void setDisp12MaxDiff( const int disp12MaxDiff );

    int getPreFilterSize() const;
    void setPreFilterSize( const int preFilterSize );

    int getPreFilterCap() const;
    void setPreFilterCap( const int preFilterCap );

    int getUniquenessRatio() const;
    void setUniquenessRatio( const int uniquenessRatio );

private:
    void initialize();

};

class StereoResult
{
public:
    StereoResult();

    void setPreviewImage( const CvImage &value );
    void setDisparity( const cv::Mat &value );
    void setPoints( const cv::Mat &value );
    void setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &value );

    const CvImage &previewImage() const;
    const cv::Mat &disparity() const;
    const CvImage &colorizedDisparity() const;
    const cv::Mat &points() const;
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointCloud() const;

    const std::chrono::time_point< std::chrono::system_clock > &time() const;

    void setFrame( const StereoFrame &frame );
    const StereoFrame &frame() const;

    const Frame &leftFrame() const;
    const Frame &rightFrame() const;

protected:
    CvImage m_previewImage;
    cv::Mat m_disparity;
    CvImage m_colorizedDisparity;
    cv::Mat m_points;
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr m_pointCloud;

    StereoFrame m_frame;

private:
    void initialize();

};

class StereoResultProcessor : public StereoProcessor
{
public:
    StereoResultProcessor();
    StereoResultProcessor( const std::shared_ptr< DisparityProcessorBase > &proc );

    void setCalibration( const StereoCalibrationDataShort &data );
    bool loadYaml( const std::string &fileName );

    StereoResult process( const StereoFrame &frame );

protected:
    StereoRectificationProcessor m_rectificationProcessor;

};
