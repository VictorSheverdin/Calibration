#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

class Map;

class MapPoint : std::enable_shared_from_this< MapPoint >
{
public:
    using MapPtr = std::weak_ptr< Map >;
    using PointPtr = std::shared_ptr< MapPoint >;

    static PointPtr create( const MapPtr &parentMap );
    static PointPtr create( const MapPtr &parentMap, const cv::Vec3f &point );
    static PointPtr create( const MapPtr &parentMap, const cv::Vec3f &point, const cv::Scalar &color );

    void setPoint( const cv::Vec3f &value );
    const cv::Vec3f &point() const;

    void setColor( const cv::Scalar &value );
    const cv::Scalar &color() const;

protected:
    MapPoint( const MapPtr &parentMap );
    MapPoint( const MapPtr &parentMap, const cv::Vec3f &point );
    MapPoint( const MapPtr &parentMap, const cv::Vec3f &point, const cv::Scalar &color );

    MapPtr m_parentMap;

    cv::Vec3f m_point;
    cv::Scalar m_color;

};

}
