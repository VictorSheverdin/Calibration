#pragma once

#include <list>
#include <memory>
#include <thread>

#include "optimizer.h"
#include "src/common/calibrationdatabase.h"
#include "src/common/colorpoint.h"

namespace slam {

class StereoFrameBase;
class MapPoint;
class World;

class Map : public std::enable_shared_from_this< Map >
{
    friend class Optimizer;

public:
    using MapPtr = std::shared_ptr< Map >;

    using FramePtr = std::shared_ptr< StereoFrameBase >;
    using MapPointPtr = std::shared_ptr< MapPoint >;

    using WorldPtr = std::shared_ptr< World >;

    static MapPtr create( const StereoCameraMatrix &cameraMatrix );

    MapPointPtr createMapPoint( const cv::Point3d &pt, const cv::Scalar &color );

    void removeMapPoint( const MapPointPtr &point );

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::set< MapPointPtr > &mapPoints();
    const std::set< MapPointPtr > &mapPoints() const;

    const FramePtr &backFrame() const;

    void addMapPoint( const MapPointPtr &point );

    const cv::Mat baselineVector() const;
    double baselineLenght() const;

    void adjust( const int frames );
    void localAdjustment();

    bool valid() const;

protected:
    using WorldPtrImpl = std::weak_ptr< World >;

    Map( const StereoCameraMatrix &projectionMatrix );

    std::list< FramePtr > m_frames;
    std::set< MapPointPtr > m_mapPoints;

    StereoCameraMatrix m_projectionMatrix;

    size_t m_previousKeypointsCount;

    Optimizer m_optimizer;

    static const int m_minTrackPoints = 1 << 6;

    static const int m_goodTrackPoints = 1 << 7;
    static const int m_overTrackPoints = 1 << 8;

    static const int m_trackFramePointsCount = 1 << 7;

    static const double m_minTrackInliersRatio;

    std::mutex m_mutex;

private:
    void initialize();

};

}
