#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

class Map;

class MapPoint : std::enable_shared_from_this< MapPoint >
{
public:
    using MapPtr = std::weak_ptr< Map >;
    using PointPtr = std::shared_ptr< MapPoint >;

    static PointPtr create( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color );

    void setPoint( const cv::Point3d &value );
    const cv::Point3d &point() const;

    void setColor( const cv::Scalar &value );
    const cv::Scalar &color() const;

protected:
    MapPoint( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color );

    MapPtr m_parentMap;

    cv::Scalar m_color;

    cv::Point3d m_point;

};

}
