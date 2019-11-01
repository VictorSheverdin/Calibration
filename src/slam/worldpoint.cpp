#include "src/common/precompiled.h"

#include "worldpoint.h"

namespace slam {

WorldPoint::WorldPoint()
{
}

WorldPoint::WorldPoint( const cv::Vec3d &point )
    : m_point( point )
{
}


}
