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
    static WorldPtr create( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::set< MapPointPtr > &mapPoints();
    const std::set< MapPointPtr > &mapPoints() const;

    CvImage keyPointsImage() const;
    CvImage stereoPointsImage() const;
    CvImage tracksImage() const;

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    const StereoCalibrationDataShort &calibration() const;

    const ProjectionMatrix &leftProjectionMatrix() const;
    const ProjectionMatrix &rightProjectionMatrix() const;

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2f &value );
    double scaleFactor() const;

    void createMap();

protected:
    World( const StereoCalibrationDataShort &calibration );
    World( const std::string &calibrationFile );
    World( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );

    StereoRectificationProcessor m_rectificationProcessor;

    ProjectionMatrix m_leftProjectionMatrix;
    ProjectionMatrix m_rightProjectionMatrix;

    MapPtr m_map;

    double m_scaleFactor;

private:
    void initialize();

};

}
