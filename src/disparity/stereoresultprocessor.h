#pragma once

#include "src/common/stereoprocessor.h"

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
