#pragma once

#include <list>
#include <memory>

#include "src/common/calibrationdatabase.h"

namespace slam {

class DoubleFrame;
class MapPoint;
class World;

class Map : public std::enable_shared_from_this< Map >
{
public:
    using MapPtr = std::shared_ptr< Map >;

    using FramePtr = std::shared_ptr< DoubleFrame >;
    using MapPointPtr = std::shared_ptr< MapPoint >;

    using WorldPtr = std::shared_ptr< World >;

    static MapPtr create( const WorldPtr &world );

    MapPointPtr createMapPoint();
    MapPointPtr createMapPoint( const cv::Vec3f &pt );
    MapPointPtr createMapPoint( const cv::Vec3f &pt, const cv::Scalar &color );

    void removeMapPoint( const MapPointPtr &point );

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::set< MapPointPtr > &mapPoints();
    const std::set< MapPointPtr > &mapPoints() const;

    const FramePtr &backFrame() const;

    CvImage keyPointsImage() const;
    CvImage stereoPointsImage() const;
    CvImage tracksImage() const;

    void addMapPoint( const MapPointPtr &point );

    WorldPtr parentWorld() const;

    const cv::Mat &baselineVector() const;
    double baselineLenght() const;

protected:
    using WorldPtrImpl = std::weak_ptr< World >;

    Map( const WorldPtr &world );

    WorldPtrImpl m_parentWorld;

    std::list< FramePtr > m_frames;
    std::set< MapPointPtr > m_mapPoints;

    CvImage m_keyPointsImage;
    CvImage m_stereoPointsImage;
    CvImage m_tracksImage;

    static const int m_minPnpPoints = 10;

    static const int m_minKeyPoints = 500;
    static const int m_maxKeyPoints = 25000;

    static const int m_goodTrackPoints = 200;
    static const int m_overageTrackPoints = 300;

    static const int m_goodStereoPoints = 400;
    static const int m_overageStereoPoints = 1000;

private:
    void initialize();

};

}
