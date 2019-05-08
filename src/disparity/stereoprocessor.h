#pragma once

#include "src/common/image.h"
#include "src/common/calibrationdata.h"

#include <QThread>
#include <QMutex>

#include <opencv/cv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DisparityProcessorBase
{
public:
    DisparityProcessorBase();

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) = 0;
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
    cv::Ptr< cv::StereoBM > m_matcher;

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
    const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &pointCloud() const;

protected:
    CvImage m_previewImage;
    cv::Mat m_disparity;
    CvImage m_colorizedDisparity;
    cv::Mat m_points;
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr m_pointCloud;

private:
    void initialize();

};

class StereoProcessor
{
public:
    StereoProcessor();

    void setCalibration( const StereoCalibrationData &data );
    const StereoCalibrationData &calibration() const;

    bool loadYaml( const std::string &fileName );

    std::shared_ptr< DisparityProcessorBase > disparityProcessor() const;
    void setDisparityProcessor( const std::shared_ptr< DisparityProcessorBase > proc );

    StereoResult process( const CvImage &leftFrame, const CvImage &rightFrame );

protected:
    std::shared_ptr< DisparityProcessorBase > m_disparityProcessor;

    StereoCalibrationData m_calibration;

private:
    void initialize();

};

class ProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit ProcessorThread( QObject *parent = nullptr );

    void queueImage( const StereoImage &img );

protected:
    std::deque< StereoImage > m_processQueue;
    QMutex m_processMutex;

    std::deque< StereoResult > m_resultQueue;
    QMutex m_resultMutex;

    static const int m_maxDequeSize = 10;


    virtual void run() override;

private:
    void initialize();

};
