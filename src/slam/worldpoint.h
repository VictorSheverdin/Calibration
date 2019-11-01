#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

class MonoFramePoint;

class WorldPoint
{
public:
    WorldPoint();
    WorldPoint( const cv::Vec3d &point );

protected:
    cv::Vec3d m_point;

    std::vector< std::shared_ptr< MonoFramePoint > > m_childFramePoints;

private:
};

}
