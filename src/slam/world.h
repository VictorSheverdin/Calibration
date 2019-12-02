#pragma once

#include "src/common/calibrationdatabase.h"
#include "src/common/rectificationprocessor.h"

#include "map.h"

namespace slam {

class World : public std::enable_shared_from_this< World >
{
public:
    using WorldPtr = std::shared_ptr< World >;

    using FramePtr = Map::FramePtr;

    using MapPtr = Map::MapPtr;
    using MapPointPtr = Map::MapPointPtr;

    static WorldPtr create( const StereoCalibrationDataShort &calibration );
    static WorldPtr create( const std::string &calibrationFile );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::set< MapPointPtr > &mapPoints();
    const std::set< MapPointPtr > &mapPoints() const;

    CvImage keyPointsImage() const;
    CvImage stereoPointsImage() const;
    CvImage tracksImage() const;

    bool track( const std::string &leftFile, const std::string &rightFile );
    bool track( const CvImage &leftImage, const CvImage &rightImage );

    const StereoCalibrationDataShort &calibration() const;

    void setScaleFactor( const double value );
    double scaleFactor() const;

    void createMap();

protected:
    World( const StereoCalibrationDataShort &calibration );
    World( const std::string &calibrationFile );

    StereoRectificationProcessor m_rectificationProcessor;

    MapPtr m_map;

    double m_scaleFactor;

private:
    void initialize();
};

}
