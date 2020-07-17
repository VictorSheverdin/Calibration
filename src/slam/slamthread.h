#pragma once

#include <QThread>
#include <QMutex>

#include <memory>

#include "slamgeometry.h"

#include "src/common/image.h"

#include "src/common/rectificationprocessor.h"

namespace slam {
    class World;
    class Map;
    class FlowMap;
    class FeatureMap;
}

class SlamThread : public QThread
{
    Q_OBJECT

public:
    using MapPtr = std::shared_ptr< slam::Map >;

    explicit SlamThread( const StereoCalibrationDataShort &calibration, QObject *parent = nullptr );

    void process( const StampedImage leftImage, const StampedImage rightImage );

    std::list< MapPtr > maps() const;

    CvImage pointsImage() const;
    CvImage tracksImage() const;
    CvImage stereoImage() const;

signals:
    void updateSignal();

protected:
    StampedImage m_leftFrame;
    StampedImage m_rightFrame;

    std::mutex m_mutex;

    double m_scaleFactor;

    std::shared_ptr< slam::World > m_system;

    StereoRectificationProcessor m_rectificationProcessor;
    MonoUndistortionProcessor m_leftUndistortionProcessor;
    MonoUndistortionProcessor m_rightUndistortionProcessor;

    virtual void run() override;

private:
    void initialize(const StereoCalibrationDataShort &calibration);

};
