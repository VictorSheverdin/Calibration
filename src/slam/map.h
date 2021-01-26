#pragma once

#include <list>
#include <memory>
#include <thread>

#include "optimizer.h"
#include "src/common/calibrationdatabase.h"
#include "src/common/colorpoint.h"

#include "alias.h"

namespace slam {

class StereoFrame;
class StereoKeyFrame;
class FlowStereoFrame;
class FlowStereoKeyFrame;
class MapPoint;
class World;

class Map : public std::enable_shared_from_this< Map >
{
public:
    using ObjectPtr = std::shared_ptr< Map >;
    using ObjectConstPtr = std::shared_ptr< const Map >;

    virtual ~Map() = default;

    static ObjectPtr create( const StereoCameraMatrix &cameraMatrix, const WorldPtr &parentWorld );

    WorldPtr parentWorld() const;

    const cv::Mat baselineVector() const;
    double baselineLenght() const;

    double minTriangulateCameraDistance() const;

    MapPointPtr createMapPoint( const cv::Point3d &pt, const cv::Scalar &color );

    void removeMapPoint( const MapPointPtr &point );

    void addMapPoint( const MapPointPtr &point );

    const std::set< MapPointPtr > &mapPoints() const;

    const std::list< StereoFramePtr > &frames() const;
    const StereoFramePtr &backFrame() const;

    bool isRudimental() const;

    StereoCameraMatrix backProjectionMatrix() const;

    bool track( const StampedImage &leftImage, const StampedImage &rightImage );


protected:
    using WorldPtrImpl = std::weak_ptr< World >;

    Map( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld );

    WorldPtrImpl m_parentWorld;

    StereoCameraMatrix m_projectionMatrix;

    std::set< MapPointPtr > m_mapPoints;

    std::list< StereoFramePtr > m_frames;

    Optimizer m_optimizer;

    static const size_t m_minTrackPoints = 70;

    static const size_t m_goodTrackPoints = 150;

    static const bool m_denseFlag = false;

    static const size_t m_adjustFramesCount = 5;

    void adjust( const int frames );
    void adjustLast();

private:
    void initialize();

};

}
