#pragma once

#include <list>
#include <memory>

#include "src/common/calibrationdatabase.h"

namespace slam {

class FrameBase;
class StereoFrame;
class MapPoint;

class Map : public std::enable_shared_from_this< Map >
{
public:
    using MapPtr = std::shared_ptr< Map >;

    using FramePtr = std::shared_ptr< FrameBase >;
    using MapPointPtr = std::shared_ptr< MapPoint >;

    static MapPtr create( const StereoCalibrationDataShort &calibration );

    MapPointPtr createMapPoint();
    MapPointPtr createMapPoint( const cv::Vec3f &pt );
    MapPointPtr createMapPoint( const cv::Vec3f &pt, const cv::Scalar &color );

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::list< MapPointPtr > &mapPoints();
    const std::list< MapPointPtr > &mapPoints() const;

    const FramePtr &backFrame() const;

    CvImage keyPointsImage() const;
    CvImage stereoPointsImage() const;
    CvImage tracksImage() const;

    void addMapPoint( const MapPointPtr &point );

protected:
    Map( const StereoCalibrationDataShort &calibration );

    std::list< FramePtr > m_frames;
    std::list< MapPointPtr > m_mapPoints;

    StereoCalibrationDataShort m_calibration;

    CvImage m_keyPointsImage;
    CvImage m_stereoPointsImage;
    CvImage m_tracksImage;

    static const int m_minPnpPoints = 10;

private:
    void initialize();

};

}
