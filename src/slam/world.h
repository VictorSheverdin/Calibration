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

    bool track( const StampedImage &leftImage, const StampedImage &rightImage );

    void setRestoreMatrix( const cv::Mat &rotation, const cv::Mat &translation );
    const cv::Mat &restoreMatrix() const;
    cv::Mat restoreRotation() const;
    cv::Mat restoreTranslation() const;

    BMStereoProcessor &stereoProcessor();
    const BMStereoProcessor &stereoProcessor() const;

    const std::unique_ptr< FlowTracker > &flowTracker() const;
    const std::unique_ptr< FeatureTracker > &featureTracker() const;

    double maxReprojectionError() const;

    double minStereoDisparity() const;

    double minAdjacentPointsDistance() const;
    double minAdjacentCameraMultiplier() const;

    double minTrackInliersRatio() const;
    double goodTrackInliersRatio() const;

    double pointsMinDistance() const;

protected:
    World( const StereoCameraMatrix &cameraMatrix );

    std::list < MapPtr > m_maps;

    StereoCameraMatrix m_startCameraMatrix;

    BMStereoProcessor m_stereoProcessor;

    std::unique_ptr< FlowTracker > m_flowTracker;
    std::unique_ptr< FeatureTracker > m_featureTracker;

    cv::Mat m_restoreMatrix;

    TrackType m_trackType;

    static const double m_maxReprojectionError;

    static const double m_minStereoDisparity;

    static const double m_minAdjacentPointsDistance;
    static const double m_minAdjacentCameraMultiplier;

    static const double m_minTrackInliersRatio;
    static const double m_goodTrackInliersRatio;

    static const double m_pointsMinDistance;

    static const size_t m_pointsCount = 1000;

    mutable std::mutex m_mutex;

    void createMap( const StereoCameraMatrix &cameraMatrix );

private:
    void initialize( const StereoCameraMatrix &cameraMatrix );

};

}
