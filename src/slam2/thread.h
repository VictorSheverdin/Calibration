#pragma once

#include <QThread>
#include <QMutex>

#include "src/common/colorpoint.h"

#include "src/common/image.h"
#include "src/common/projectionmatrix.h"
#include "src/common/limitedqueue.h"

#include "parameters.h"

#include "alias.h"

class ProcessorThread : public QThread
{
    Q_OBJECT

    friend class SlamImageWidget;

public:
    explicit ProcessorThread( const slam2::Parameters &parameters, QObject *parent = nullptr );

    void process( const StampedStereoImage image );

    slam2::SystemPtr system() const;

    CvImage pointsImage() const;
    CvImage tracksImage() const;
    CvImage stereoImage() const;

    std::vector< ColorPoint3d > sparseCloud() const;

    std::vector< StereoProjectionMatrix > path() const;

signals:
    void updateSignal();

protected:
    LimitedQueue< StampedStereoImage > _processQueue;

    QMutex _queueMutex;
    mutable QMutex _resultMutex;

    CvImage _pointsImage;
    CvImage _tracksImage;
    CvImage _stereoImage;

    std::vector< ColorPoint3d > _sparseCloud;

    std::vector< cv::Point3d > _startingPoints;

    std::vector< StereoProjectionMatrix > _path;

    slam2::SystemPtr _system;

    virtual void run() override;

    void processNext();

};
