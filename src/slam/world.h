#pragma once

#include "map.h"
#include "tracker.h"
#include "src/common/stereoprocessor.h"

namespace slam {

class World : public std::enable_shared_from_this< World >
{
public:
    using WorldPtr = std::shared_ptr< World >;

    using FramePtr = FeatureMap::FramePtr;

    using MapPtr = FeatureMap::MapPtr;
    using MapPointPtr = FeatureMap::MapPointPtr;

    static WorldPtr create( const StereoCameraMatrix &cameraMatrix );

    std::list<MapPtr> maps() const;

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    void adjust( const int frames );
    void localAdjustment();

    BMStereoProcessor &stereoProcessor();
    const BMStereoProcessor &stereoProcessor() const;

    GPUFlowTracker &flowTracker();
    const GPUFlowTracker &flowTracker() const;

    const std::unique_ptr< FeatureTracker > &featureTracker() const;

    double maxReprojectionError() const;

    double minStereoXDisparity() const;
    double maxStereoYDisparity() const;

    double minAdjacentPointsDistance() const;
    double minAdjacentCameraMultiplier() const;

protected:
    World( const StereoCameraMatrix &cameraMatrix );

    std::list < MapPtr > m_maps;

    StereoCameraMatrix m_startCameraMatrix;

    BMStereoProcessor m_stereoProcessor;

    GPUFlowTracker m_flowTracker;
    std::unique_ptr< FeatureTracker > m_featureTracker;

    static const int m_keypointsCount = 100000;

    static const double m_maxReprojectionError;

    static const double m_minStereoXDisparity;
    static const double m_maxStereoYDisparity;

    static const double m_minAdjacentPointsDistance;
    static const double m_minAdjacentCameraMultiplier;

    mutable std::mutex m_mutex;

private:
    void initialize( const StereoCameraMatrix &cameraMatrix );

};

}
