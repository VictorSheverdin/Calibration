#pragma once

#include "map.h"
#include "tracker.h"
#include "src/common/stereoprocessor.h"

namespace slam {

class World : public std::enable_shared_from_this< World >
{
public:
    enum class TrackType { FLOW, FEATURES };

    using ObjectPtr = std::shared_ptr< World >;

    using FramePtr = FeatureMap::FramePtr;

    using MapPtr = std::shared_ptr< Map >;
    using MapPointPtr = FeatureMap::MapPointPtr;

    static ObjectPtr create( const StereoCameraMatrix &cameraMatrix );

    std::list< MapPtr > maps() const;

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    void adjust( const int frames );
    void localAdjustment();

    BMStereoProcessor &stereoProcessor();
    const BMStereoProcessor &stereoProcessor() const;

    const std::unique_ptr< FlowTracker > &flowTracker() const;
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

    std::unique_ptr< FlowTracker > m_flowTracker;
    std::unique_ptr< FeatureTracker > m_featureTracker;

    TrackType m_trackType;

    static const double m_maxReprojectionError;

    static const double m_minStereoXDisparity;
    static const double m_maxStereoYDisparity;

    static const double m_minAdjacentPointsDistance;
    static const double m_minAdjacentCameraMultiplier;

    mutable std::mutex m_mutex;

    void createMap( const StereoCameraMatrix &cameraMatrix );

private:
    void initialize( const StereoCameraMatrix &cameraMatrix );

};

}
