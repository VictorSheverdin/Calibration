#pragma once

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

#include "colorpoint.h"

namespace slam {

class Map;
class MonoPoint;

class MapPoint : public ColorPoint3d, public std::enable_shared_from_this< MapPoint >
{
    friend class Map;

public:
    using MapPtr = std::weak_ptr< Map >;
    using PointPtr = std::shared_ptr< MapPoint >;

    using FramePointPtr = std::shared_ptr< MonoPoint >;
    using FrameConstPointPtr = std::shared_ptr< const MonoPoint >;

    static PointPtr create( const MapPtr &parentMap, const cv::Point3d &point, const cv::Scalar &color );

    void addFramePoint( const FramePointPtr &value );
    void removeFramePoint( const FramePointPtr &value );

    bool isFramePoint( const FrameConstPointPtr &value ) const;

    bool isLastFramePoint( const FrameConstPointPtr &value ) const;

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
