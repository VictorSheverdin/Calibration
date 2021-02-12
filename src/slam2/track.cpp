#include "src/common/precompiled.h"

#include "track.h"

#include "framepoint.h"
#include "mappoint.h"

namespace slam2 {

// Track
Track::Track()
{
    initialize();
}

void Track::initialize()
{
    _maxIndex = 0;
}

Track::ObjectPtr Track::create()
{
    return ObjectPtr( new Track() );;
}

void Track::addPoint( const Point2Ptr &point )
{
    setPoint( _maxIndex, point );

    ++_maxIndex;
}

void Track::setPoint( const size_t index, const Point2Ptr &point )
{
    _points[ index ] = point;

    point->setParentTrack( shared_from_this() );
    point->setTrackIndex( index );
}

std::vector< Point2Ptr > Track::pointsVector() const
{
    std::vector< Point2Ptr > ret;

    ret.reserve( _points.size() );

    for ( auto &i : _points )
        ret.push_back( i.second );

    return ret;
}

const std::map< size_t, Point2Ptr > &Track::points() const
{
    return _points;
}

void Track::setMapPoint( const MapPointPtr &value )
{
    _mapPoint = value;
}

MapPointPtr Track::mapPoint() const
{
    return _mapPoint;
}

void Track::createMapPoint( const ColorPoint3d &point )
{
    _mapPoint = MapPoint::create( point, shared_from_this() );
}

void Track::clearMapPoint()
{
    _mapPoint.reset();
}

}
