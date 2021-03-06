#include "src/common/precompiled.h"

#include "mappoint.h"

namespace slam2 {

// MapPoint
MapPoint::MapPoint( const ColorPoint3d &point, const TrackPtr &parent )
    : Parent_Weak_Ptr< Track >( parent )
{
    setPoint( point );
}

MapPoint::ObjectPtr MapPoint::create( const ColorPoint3d &point, const TrackPtr &parent )
{
    return ObjectPtr( new MapPoint( point, parent ) );
}

void MapPoint::setPoint( const ColorPoint3d &point )
{
    _point = point;
}

const ColorPoint3d &MapPoint::point() const
{
    return _point;
}

}
