#pragma once

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

#include "src/common/colorpoint.h"

#include "alias.h"

namespace slam {

class Map;
class MonoPoint;

class MapPoint : public ColorPoint3d, public std::enable_shared_from_this< MapPoint >
{
public:
    using ObjectPtr = std::shared_ptr< MapPoint >;
    using ObjectConstPtr = std::shared_ptr< const MapPoint >;

    static ObjectPtr create( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color );

    void addFramePoint( const MonoPointPtr &value );
    void removeFramePoint( const MonoPointPtr &value );

    bool isFramePoint( const MonoPointPtr &value ) const;

    bool isLastFramePoint( const MonoPointPtr &value ) const;

    void setEigenPoint( const Eigen::Matrix< double, 3, 1 > &value );
    Eigen::Matrix< double, 3, 1 > eigenPoint() const;

protected:
    using FramePointPtrImpl = std::weak_ptr< MonoPoint >;

    MapPoint( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color );

    MapPtr m_parentMap;

    std::list< FramePointPtrImpl > m_framePoints;

private:
    void initialize();

};

}
