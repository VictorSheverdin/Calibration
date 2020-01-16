#include "src/common/precompiled.h"

#include "colorpoint.h"

// ColorPointBase
ColorPointBase::ColorPointBase()
{
}

ColorPointBase::ColorPointBase( const cv::Scalar &color )
{
    setColor( color );
}

void ColorPointBase::setColor( const cv::Scalar &color )
{
    m_color = color;
}

const cv::Scalar &ColorPointBase::color() const
{
    return m_color;
}

// ColorPoint2d
ColorPoint2d::ColorPoint2d()
{
}

ColorPoint2d::ColorPoint2d( const cv::Point2f &point, const cv::Scalar &color )
    : ColorPointBase( color )
{
    setPoint( point );
}

void ColorPoint2d::setPoint( const cv::Point2f &point )
{
    m_point = point;
}

const cv::Point2f &ColorPoint2d::point() const
{
    return m_point;
}

void ColorPoint2d::set( const cv::Point2f &point, const cv::Scalar &color )
{
    setPoint( point );
    setColor( color );
}

// ColorPoint3d
ColorPoint3d::ColorPoint3d()
{
}

ColorPoint3d::ColorPoint3d( const cv::Point3f &point, const cv::Scalar &color )
    : ColorPointBase( color )
{
    setPoint( point );
}

void ColorPoint3d::setPoint( const cv::Point3f &point )
{
    m_point = point;
}

const cv::Point3f &ColorPoint3d::point() const
{
    return m_point;
}

void ColorPoint3d::set( const cv::Point3f &point, const cv::Scalar &color )
{
    setPoint( point );
    setColor( color );
}
