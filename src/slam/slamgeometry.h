#pragma once

#include <QObject>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "src/common/projectionmatrix.h"

class SlamGeometry
{
public:
    SlamGeometry();

    void setPoints( pcl::PointCloud< pcl::PointXYZRGB >::Ptr &ptr );
    const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &points() const;

    void addPath( const StereoCameraMatrix &matrix );

protected:
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr m_points;

    std::list< StereoCameraMatrix > m_path;

};

Q_DECLARE_METATYPE( SlamGeometry )
