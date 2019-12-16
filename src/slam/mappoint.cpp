#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"

#include "framepoint.h"

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

void MapPoint::addFramePoint( const FramePointPtr &value )
{
    if ( !isFramePoint( value ))
        m_framePoints.push_back( value );
}

void MapPoint::removeFramePoint( const FramePointPtr &value )
{
    for ( auto i = m_framePoints.begin(); i != m_framePoints.end(); ) {
        if ( i->expired() )
            i = m_framePoints.erase( i );
        else {
            if ( i->lock() == value )
                i = m_framePoints.erase( i );
            else
                ++i;
        }

    }

}

bool MapPoint::isFramePoint( const FramePointPtr &value ) const
{
    for ( auto i = m_framePoints.begin(); i != m_framePoints.end(); ++i )
        if ( !i->expired() && i->lock() == value )
            return true;

    return false;

}

bool MapPoint::isLastFramePoint( const FramePointPtr &value ) const
{
    if ( !m_framePoints.empty() )
        if ( !m_framePoints.back().expired() && m_framePoints.back().lock() == value )
            return true;

    return false;

}

}
