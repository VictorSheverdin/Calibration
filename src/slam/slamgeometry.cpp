#include "src/common/precompiled.h"

#include "slamgeometry.h"

SlamGeometry::SlamGeometry()
{
}

void SlamGeometry::setPoints( pcl::PointCloud< pcl::PointXYZRGB >::Ptr &ptr )
{
    m_points = ptr;
}

const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &SlamGeometry::points() const
{
    return m_points;
}

void SlamGeometry::addPath( const StereoCameraMatrix &matrix )
{
    m_path.push_back( matrix );
}

