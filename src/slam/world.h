#pragma once

#include "src/common/calibrationdatabase.h"
#include "src/common/rectificationprocessor.h"

#include "map.h"

namespace slam {

class World
{
public:
    using FramePtr = Map::FramePtr;

    using MapPtr = Map::MapPtr;
    using MapPointPtr = Map::MapPointPtr;

    World( const StereoCalibrationDataShort &calibration );
    World( const std::string &calibrationFile );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::list< MapPointPtr > &mapPoints();
    const std::list< MapPointPtr > &mapPoints() const;

    CvImage keyPointsImage() const;
    CvImage stereoPointsImage() const;
    CvImage tracksImage() const;

    bool track( const std::string &leftFile, const std::string &rightFile );
    bool track( const CvImage &leftImage, const CvImage &rightImage );

protected:
    StereoRectificationProcessor m_rectificationProcessor;
    MonoUndistortionProcessor m_leftUndistortionProcessor;
    MonoUndistortionProcessor m_rightUndistortionProcessor;

    MapPtr m_map;

private:
    void initialize();
};

}
