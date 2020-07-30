#include "src/common/precompiled.h"

#include "slamgeometry.h"

SlamGeometry::SlamGeometry()
{
}

void SlamGeometry::addPath( const StereoCameraMatrix &matrix )
{
    m_path.push_back( matrix );
}

void SlamGeometry::addPoint( const ColorPoint3d &point )
{
    m_points.push_back( point );
}

void SlamGeometry::addPoints( const std::list< ColorPoint3d > &points )
{
    for ( auto &i : points )
        addPoint( i );
}

const std::list< StereoCameraMatrix > &SlamGeometry::path() const
{
    return m_path;
}

const std::list< ColorPoint3d > &SlamGeometry::points() const
{
    return m_points;
}
