#pragma once

#include <QThread>
#include <QMutex>

#include "src/common/image.h"

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

signals:
    void updateSignal();

protected:
    std::list< StampedStereoImage > _processQueue;

    QMutex _queueMutex;
    mutable QMutex _resultMutex;

    CvImage _pointsImage;
    CvImage _tracksImage;
    CvImage _stereoImage;

    slam2::SystemPtr _system;

    virtual void run() override;

    void processNext();

};
