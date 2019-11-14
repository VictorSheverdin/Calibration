#include "pointtrack.h"

#include "framepoint.h"

namespace slam {

PointTrack::PointTrack()
{
}

PointTrack::PointTrack( const WorldPointPtr worldPoint )
{
    setWorldPoint( worldPoint );
}

PointTrack::TrackPtr PointTrack::create()
{
    return TrackPtr( new PointTrack() );
}

PointTrack::TrackPtr PointTrack::create( const WorldPointPtr worldPoint )
{
    return TrackPtr( new PointTrack( worldPoint ) );
}

void PointTrack::setWorldPoint( const WorldPointPtr value )
{
    m_parentWorldPoint = value;
}

std::list< PointTrack::MonoPointPtr > &PointTrack::points()
{
    return m_points;
}

const std::list< PointTrack::MonoPointPtr > &PointTrack::points() const
{
    return m_points;
}

void PointTrack::addPoint( const MonoPointPtr point )
{
    m_points.push_back( point );
}

}
