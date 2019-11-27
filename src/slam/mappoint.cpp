#include "src/common/precompiled.h"

#include "map.h"
#include "mappoint.h"

namespace slam {

// MapPoint
MapPoint::MapPoint( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
}

MapPoint::MapPoint( const MapPtr &parentMap, const cv::Vec3f &point )
    : m_parentMap( parentMap ), m_point( point )
{
}

MapPoint::MapPoint( const MapPtr &parentMap, const cv::Vec3f &point, const cv::Scalar &color )
    : m_parentMap( parentMap ), m_point( point ), m_color( color )
{
}

MapPoint::PointPtr MapPoint::create( const MapPtr &parentMap )
{
    return PointPtr( new MapPoint( parentMap ) );
}

MapPoint::PointPtr MapPoint::create( const MapPtr &parentMap, const cv::Vec3f &point )
{
    return PointPtr( new MapPoint( parentMap, point ) );
}

MapPoint::PointPtr MapPoint::create( const MapPtr &parentMap, const cv::Vec3f &point, const cv::Scalar &color )
{
    return PointPtr( new MapPoint( parentMap, point, color ) );
}

void MapPoint::setPoint( const cv::Vec3f &value )
{
    m_point = value;
}

const cv::Vec3f &MapPoint::point() const
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
