#pragma once

#include "src/common/image.h"
#include "src/common/calibrationdatabase.h"

#include <QThread>
#include <QMutex>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
    void setMode(int mode);

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

class StereoProcessor
{
public:
    StereoProcessor();

    void setCalibration( const StereoCalibrationDataShort &data );
    const StereoCalibrationDataShort &calibration() const;

    bool loadYaml( const std::string &fileName );

    std::shared_ptr< DisparityProcessorBase > disparityProcessor() const;
    void setDisparityProcessor( const std::shared_ptr< DisparityProcessorBase > proc );

    StereoResult process( const StereoFrame &frame );

protected:
    std::shared_ptr< DisparityProcessorBase > m_disparityProcessor;

    StereoCalibrationDataShort m_calibration;

private:
    void initialize();

};

class ProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit ProcessorThread( QObject *parent = nullptr );

    bool process( const StereoFrame &frame );

    void setProcessor( const std::shared_ptr< StereoProcessor > processor );

    StereoResult result();

signals:
    void frameProcessed();

protected:
    StereoFrame m_frame;
    QMutex m_processMutex;

    StereoResult m_result;
    QMutex m_resultMutex;

    std::shared_ptr< StereoProcessor > m_processor;

    virtual void run() override;

private:
    void initialize();

};
