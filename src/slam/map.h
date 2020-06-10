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
public:
    using WorldPtr = std::shared_ptr< World >;
    using FramePtr = std::shared_ptr< StereoFrameBase >;
    using MapPointPtr = std::shared_ptr< MapPoint >;

    virtual ~Map() = default;

    WorldPtr parentWorld() const;

    const cv::Mat baselineVector() const;
    double baselineLenght() const;

    double minTriangulateCameraDistance() const;

    MapPointPtr createMapPoint( const cv::Point3d &pt, const cv::Scalar &color );

    void removeMapPoint( const MapPointPtr &point );

    void addMapPoint( const MapPointPtr &point );

    std::set< MapPointPtr > mapPoints() const;

    std::list< FramePtr > frames() const;
    const FramePtr &backFrame() const;

    void adjust( const int frames );
    void localAdjustment();

    bool isRudimental() const;

    virtual bool track( const CvImage &leftImage, const CvImage &rightImage ) = 0;

protected:
    using WorldPtrImpl = std::weak_ptr< World >;

    Map( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld );

    WorldPtrImpl m_parentWorld;

    StereoCameraMatrix m_projectionMatrix;

    std::set< MapPointPtr > m_mapPoints;

    std::list< FramePtr > m_frames;

    Optimizer m_optimizer;

    mutable std::mutex m_mutex;

    static const double m_minTriangulateDistanceMultiplier;

    static const int m_minTrackPoints = 1 << 6;

    static const int m_goodTrackPoints = 1 << 7;
    static const int m_overTrackPoints = 1 << 8;

    static const int m_trackFramePointsCount = 1 << 7;

    static const double m_minTrackInliersRatio;
    static const double m_goodTrackInliersRatio;

    size_t m_previousKeypointsCount;

private:
    void initialize();

};

class FlowMap : public Map
{
public:
    using ObjectPtr = std::shared_ptr< FlowMap >;

    static ObjectPtr create( const StereoCameraMatrix &cameraMatrix, const WorldPtr &parentWorld );

    std::shared_ptr< FlowMap > shared_from_this();
    std::shared_ptr< const FlowMap > shared_from_this() const;

    virtual bool track( const CvImage &leftImage, const CvImage &rightImage ) override;

protected:
    FlowMap( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld );

};

class FeatureMap : public Map
{
public:
    using ObjectPtr = std::shared_ptr< FeatureMap >;

    static ObjectPtr create( const StereoCameraMatrix &cameraMatrix, const WorldPtr &parentWorld );

    std::shared_ptr< FeatureMap > shared_from_this();
    std::shared_ptr< const FeatureMap > shared_from_this() const;

    virtual bool track( const CvImage &leftImage, const CvImage &rightImage ) override;

protected:
    FeatureMap( const StereoCameraMatrix &projectionMatrix, const WorldPtr &parentWorld );

};

}
