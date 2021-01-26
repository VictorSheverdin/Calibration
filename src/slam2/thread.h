#pragma once

#include <QThread>
#include <QMutex>

#include "src/common/image.h"

#include "alias.h"

class ProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit ProcessorThread( QObject *parent = nullptr );

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
    QMutex _resultMutex;

    slam2::SystemPtr _system;

    virtual void run() override;

};
