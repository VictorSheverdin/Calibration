#include "src/common/precompiled.h"

#include "track.h"

#include "framepoint.h"

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
    _points[ _maxIndex ] = point;

    point->setParentTrack( shared_from_this() );
    point->setTrackIndex( _maxIndex );

    ++_maxIndex;
}

std::vector< Point2Ptr > Track::validPoints() const
{
    std::vector< Point2Ptr > ret;

    ret.reserve( _points.size() );

    for ( auto &i : _points )
        if ( !i.second.expired() )
            ret.push_back( i.second.lock() );

    return ret;
}

}
