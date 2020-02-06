#pragma once

#include "src/common/stereoprocessor.h"
#include "src/common/rectificationprocessor.h"

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

class ProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit ProcessorThread( QObject *parent = nullptr );

    bool process( const StereoFrame &frame );

    void setProcessor( const std::shared_ptr< StereoResultProcessor > processor );

    StereoResult result();

signals:
    void frameProcessed();

protected:
    StereoFrame m_frame;
    QMutex m_processMutex;

    StereoResult m_result;
    QMutex m_resultMutex;

    std::shared_ptr< StereoResultProcessor > m_processor;

    virtual void run() override;

private:
    void initialize();

};
