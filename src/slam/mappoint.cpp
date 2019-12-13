#include "src/common/precompiled.h"

#include "map.h"
#include "mappoint.h"

namespace slam {

// MapPoint
MapPoint::MapPoint( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color )
    : m_parentMap( parentMap )
{
    setPoint( point );
    setColor( color );
}

MapPoint::PointPtr MapPoint::create( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color)
{
    return PointPtr( new MapPoint( parentMap, point, color ) );
}

void MapPoint::setPoint( const cv::Point3d &value )
{
    m_point = value;
}

const cv::Point3d &MapPoint::point() const
{
    return m_point;
}

void MapPoint::setColor( const cv::Scalar &value )
{
    m_color = value;
}

const cv::Scalar &MapPoint::color() const
{
    return m_color;
}

}
