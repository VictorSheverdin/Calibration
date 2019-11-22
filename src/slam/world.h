#pragma once

#include <list>
#include <memory>

#include "src/common/calibrationdatabase.h"

namespace slam {

class FrameBase;
class StereoFrame;
class WorldPoint;

class World : public std::enable_shared_from_this< World >
{
public:
    using WorldPtr = std::shared_ptr< World >;

    using FramePtr = std::shared_ptr< StereoFrame >;
    using WorldPointPtr = std::shared_ptr< WorldPoint >;

    static WorldPtr create( const StereoCalibrationDataShort &calibration );

    WorldPointPtr createWorldPoint();
    WorldPointPtr createWorldPoint( const cv::Vec3f &pt );
    WorldPointPtr createWorldPoint( const cv::Vec3f &pt, const cv::Scalar &color );

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::list< WorldPointPtr > &worldPoints();
    const std::list< WorldPointPtr > &worldPoints() const;

    const FramePtr &backFrame() const;

    CvImage keyPointsImage() const;
    CvImage stereoPointsImage() const;
    CvImage tracksImage() const;

    void addWorldPoint( const WorldPointPtr &point );

protected:
    World( const StereoCalibrationDataShort &calibration );

    std::list< FramePtr > m_frames;
    std::list< WorldPointPtr > m_worldPoints;

    StereoCalibrationDataShort m_calibration;

    CvImage m_keyPointsImage;
    CvImage m_stereoPointsImage;
    CvImage m_tracksImage;

    static const int m_minPnpPoints = 10;

private:
    void initialize();

};

}
