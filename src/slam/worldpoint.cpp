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

WorldPoint::PointPtr WorldPoint::create( const WorldPtr &parentWorld )
{
    return PointPtr( new WorldPoint( parentWorld ) );
}

WorldPoint::PointPtr WorldPoint::create( const WorldPtr &parentWorld, const cv::Vec3f &point )
{
    return PointPtr( new WorldPoint( parentWorld, point ) );
}


}
