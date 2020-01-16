#pragma once

#include <opencv2/opencv.hpp>

class ColorPointBase
{
public:
    ColorPointBase();
    ColorPointBase( const cv::Scalar &color );

    void setColor( const cv::Scalar &color );
    const cv::Scalar &color() const;

protected:
    cv::Scalar m_color;
};

class ColorPoint2d : public ColorPointBase
{
public:
    ColorPoint2d();
    ColorPoint2d( const cv::Point2f &point, const cv::Scalar &color );

    void setPoint( const cv::Point2f &point );
    const cv::Point2f &point() const;

    void set( const cv::Point2f &point, const cv::Scalar &color );

protected:
    cv::Point2f m_point;

};

class ColorPoint3d : public ColorPointBase
{
public:
    ColorPoint3d();
    ColorPoint3d( const cv::Point3f &point, const cv::Scalar &color );

    void setPoint( const cv::Point3f &point );
    const cv::Point3f &point() const;

    void set( const cv::Point3f &point, const cv::Scalar &color );

protected:
    cv::Point3f m_point;

};
