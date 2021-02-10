#pragma once

#include <QObject>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "src/common/projectionmatrix.h"

#include "src/common/colorpoint.h"

class SlamGeometry
{
public:
    SlamGeometry();

    void addPath( const StereoProjectionMatrix &matrix );
    void addPoint( const ColorPoint3d &point );
    void addPoints( const std::list< ColorPoint3d > &points );

    const std::list< StereoProjectionMatrix > &path() const;
    const std::list< ColorPoint3d > &points() const;

protected:
    std::list< ColorPoint3d > m_points;
    std::list< StereoProjectionMatrix > m_path;

};

Q_DECLARE_METATYPE( SlamGeometry )
