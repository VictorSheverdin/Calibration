#pragma once

#include <QThread>
#include <QMutex>

#include "src/common/templateprocessor.h"
#include "src/common/markerprocessor.h"

class MonocularProcessorThread : public QThread
{
    Q_OBJECT

public:
    enum Type { NONE, TEMPLATE, MARKER };

    explicit MonocularProcessorThread( QObject *parent = nullptr );

    const TemplateProcessor &templateProcessor() const;
    TemplateProcessor &templateProcessor();

    const ArucoProcessor &markerProcessor() const;
    ArucoProcessor &markerProcessor();

    void processFrame( const Frame &frame );

signals:
    void updateSignal();

protected:
    TemplateProcessor m_templateProcessor;
    ArucoProcessor m_markerProcessor;

    Frame m_frame;
    Type m_type;
    QMutex m_mutex;

    CvImage m_preview;
    std::vector< cv::Point2f > m_imagePoints;
    std::vector< cv::Point2f > m_worldPoints;

    virtual void run() override;

private:
    void initialize();

};

class StereoProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit StereoProcessorThread( QObject *parent = nullptr );

protected:
    virtual void run() override;

private:
    void initialize();

};
