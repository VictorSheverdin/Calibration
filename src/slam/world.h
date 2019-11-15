#pragma once

#include <list>
#include <memory>

#include "src/common/calibrationdatabase.h"

namespace slam {

class FrameBase;
class WorldPoint;

class World : public std::enable_shared_from_this< World >
{
public:
    using WorldPtr = std::shared_ptr< World >;

    using FramePtr = std::shared_ptr< FrameBase >;
    using WorldPointPtr = std::shared_ptr< WorldPoint >;

    static WorldPtr create( const StereoCalibrationDataShort &calibration );

    WorldPointPtr createWorldPoint();
    WorldPointPtr createWorldPoint( const cv::Vec3f &pt );

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    std::vector< WorldPointPtr > &worldPoints();
    const std::vector< WorldPointPtr > &worldPoints() const;

    void addWorldPoint( const WorldPointPtr &point );

protected:
    World( const StereoCalibrationDataShort &calibration );

    std::list< FramePtr > m_frames;
    std::vector< WorldPointPtr > m_worldPoints;

    StereoCalibrationDataShort m_calibration;

private:
    void initialize();

};

}
