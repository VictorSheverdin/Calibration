#pragma once

#include "map.h"
#include "tracker.h"
#include "src/common/stereoprocessor.h"

#include "settings.h"

namespace slam {

class World : public std::enable_shared_from_this< World >
{
public:
    using ObjectPtr = std::shared_ptr< World >;
    using ObjectConstPtr = std::shared_ptr< const World >;

    static ObjectPtr create( const StereoProjectionMatrix &cameraMatrix );

    const std::list< MapPtr > &maps() const;

    bool track( const StampedImage &leftImage, const StampedImage &rightImage );

    void setRestoreMatrix( const cv::Mat &rotation, const cv::Mat &translation );
    const cv::Mat &restoreMatrix() const;
    cv::Mat restoreRotation() const;
    cv::Mat restoreTranslation() const;

    BMStereoProcessor &stereoProcessor();
    const BMStereoProcessor &stereoProcessor() const;

    const std::unique_ptr< FlowTracker > &flowTracker() const;
    const std::unique_ptr< FeatureTracker > &featureTracker() const;

    const Settings &settings() const;

    double maxReprojectionError() const;

    double minStereoDisparity() const;

    double minAdjacentPointsDistance() const;
    double minAdjacentCameraMultiplier() const;

    double minTrackInliersRatio() const;
    double goodTrackInliersRatio() const;

    CvImage pointsImage() const;
    CvImage tracksImage() const;
    CvImage stereoImage() const;

    std::list< StereoProjectionMatrix > path() const;
    std::vector< ColorPoint3d > sparseCloud() const;

protected:
    World( const StereoProjectionMatrix &cameraMatrix );

    std::list < MapPtr > m_maps;

    StereoProjectionMatrix m_startCameraMatrix;

    BMStereoProcessor m_stereoProcessor;

    std::unique_ptr< FlowTracker > m_flowTracker;
    std::unique_ptr< FeatureTracker > m_featureTracker;

    cv::Mat m_restoreMatrix;

    Settings m_settings;

    void createMap( const StereoProjectionMatrix &cameraMatrix );

private:
    void initialize( const StereoProjectionMatrix &cameraMatrix );

};

}
