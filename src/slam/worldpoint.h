#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

class World;

class WorldPoint : std::enable_shared_from_this< WorldPoint >
{
public:
    using WorldPtr = std::weak_ptr< World >;
    using PointPtr = std::shared_ptr< WorldPoint >;

    static PointPtr create( const WorldPtr &parentWorld );
    static PointPtr create( const WorldPtr &parentWorld, const cv::Vec3f &point );
    static PointPtr create( const WorldPtr &parentWorld, const cv::Vec3f &point, const cv::Scalar &color );

    void setPoint( const cv::Vec3f &value );
    const cv::Vec3f &point() const;

    void setColor( const cv::Scalar &value );
    const cv::Scalar &color() const;

protected:
    WorldPoint( const WorldPtr &parentWorld );
    WorldPoint( const WorldPtr &parentWorld, const cv::Vec3f &point );
    WorldPoint( const WorldPtr &parentWorld, const cv::Vec3f &point, const cv::Scalar &color );

    WorldPtr m_parentWorld;

    cv::Vec3f m_point;
    cv::Scalar m_color;

};

}
