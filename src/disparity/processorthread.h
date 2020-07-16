#pragma once

#include "stereoresultprocessor.h"
#include "src/common/rectificationprocessor.h"

#include <QThread>
#include <QMutex>

class ProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit ProcessorThread( QObject *parent = nullptr );

    bool process( const StampedStereoImage &frame );

    void setProcessor( const std::shared_ptr< StereoResultProcessor > processor );

    StereoResult result();

signals:
    void frameProcessed();

protected:
    StampedStereoImage m_frame;
    QMutex m_processMutex;

    StereoResult m_result;
    QMutex m_resultMutex;

    std::shared_ptr< StereoResultProcessor > m_processor;

    virtual void run() override;

private:
    void initialize();

};
