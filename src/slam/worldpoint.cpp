#include "src/common/precompiled.h"

#include "worldpoint.h"
#include "world.h"

namespace slam {

// WorldPoint
WorldPoint::WorldPoint( const WorldPtr &parentWorld )
    : m_parentWorld( parentWorld )
{
}

WorldPoint::WorldPoint( const WorldPtr &parentWorld, const cv::Vec3f &point )
    : m_parentWorld( parentWorld ), m_point( point )
{
}

WorldPoint::WorldPoint( const WorldPtr &parentWorld, const cv::Vec3f &point, const cv::Scalar &color )
    : m_parentWorld( parentWorld ), m_point( point ), m_color( color )
{
}

WorldPoint::PointPtr WorldPoint::create( const WorldPtr &parentWorld )
{
    return PointPtr( new WorldPoint( parentWorld ) );
}

WorldPoint::PointPtr WorldPoint::create( const WorldPtr &parentWorld, const cv::Vec3f &point )
{
    return PointPtr( new WorldPoint( parentWorld, point ) );
}

WorldPoint::PointPtr WorldPoint::create( const WorldPtr &parentWorld, const cv::Vec3f &point, const cv::Scalar &color )
{
    return PointPtr( new WorldPoint( parentWorld, point, color ) );
}

void WorldPoint::setPoint( const cv::Vec3f &value )
{
    m_point = value;
}

const cv::Vec3f &WorldPoint::point() const
{
    return m_point;
}

void WorldPoint::setColor( const cv::Scalar &value )
{
    m_color = value;
}

const cv::Scalar &WorldPoint::color() const
{
    return m_color;
}


}
