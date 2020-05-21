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
}

class SlamThread : public QThread
{
    Q_OBJECT

public:
    using MapPtr = std::shared_ptr< slam::Map >;

    explicit SlamThread( const StereoCalibrationDataShort &calibration, QObject *parent = nullptr );

    void process( const CvImage leftImage, const CvImage rightImage );

    std::list< MapPtr > maps() const;

    CvImage pointsImage() const;
    CvImage tracksImage() const;
    CvImage stereoImage() const;

signals:
    void updateSignal();

protected:
    CvImage m_leftFrame;
    CvImage m_rightFrame;

    mutable QMutex m_mutex;

    double m_scaleFactor;

    std::shared_ptr< slam::World > m_system;

    StereoRectificationProcessor m_rectificationProcessor;

    virtual void run() override;

private:
    void initialize(const StereoCalibrationDataShort &calibration);

};
