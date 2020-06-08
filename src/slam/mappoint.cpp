#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"

#include "framepoint.h"

#include "optimizer.h"

namespace slam {

// MapPoint
MapPoint::MapPoint( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color )
    : ColorPoint3d( point, color ), m_parentMap( parentMap )
{
    initialize();
}

void MapPoint::initialize()
{
}

MapPoint::ObjectPtr MapPoint::create( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color)
{
    return ObjectPtr( new MapPoint( parentMap, point, color ) );
}

void MapPoint::setEigenPoint( const Eigen::Matrix< double, 3, 1 > &value )
{
    cv::Point3d point;

    point.x = value( 0 );
    point.y = value( 1 );
    point.z = value( 2 );

    setPoint( point );

}

Eigen::Matrix< double, 3, 1 > MapPoint::eigenPoint() const
{
    Eigen::Matrix< double, 3, 1 > ret;

    auto point = this->point();

    ret << point.x, point.y, point.z;

    return ret;
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

bool MapPoint::isFramePoint( const FrameConstPointPtr &value ) const
{
    for ( auto &i : m_framePoints )
        if ( !i.expired() && i.lock() == value )
            return true;

    return false;

}

bool MapPoint::isLastFramePoint( const FrameConstPointPtr &value ) const
{
    if ( !m_framePoints.empty() )
        if ( !m_framePoints.back().expired() && m_framePoints.back().lock() == value )
            return true;

    return false;

}

}
