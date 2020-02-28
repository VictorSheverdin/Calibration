#pragma once

#include <QThread>
#include <QMutex>

#include "src/common/templateprocessor.h"
#include "src/common/markerprocessor.h"

struct MonocularProcessorResult
{
    Frame sourceFrame;
    bool exist;
    CvImage preview;
    std::vector< cv::Point2f > imagePoints;
    std::vector< cv::Point3f > worldPoints;
};

struct StereoProcessorResult
{
    StereoFrame sourceFrame;
    bool leftExist;
    bool rightExist;
    CvImage leftPreview;
    CvImage rightPreview;
    std::vector< cv::Point2f > leftImagePoints;
    std::vector< cv::Point2f > rightImagePoints;
    std::vector< cv::Point3f > worldPoints;
};

class ProcessorThreadBase : public QThread
{
    Q_OBJECT

public:
    enum Type { NONE, TEMPLATE, MARKER };

    explicit ProcessorThreadBase( QObject *parent = nullptr );

    const TemplateProcessor &templateProcessor() const;
    TemplateProcessor &templateProcessor();

    const ArucoProcessor &markerProcessor() const;
    ArucoProcessor &markerProcessor();

signals:
    void updateSignal();

protected:
    Type m_type;

    TemplateProcessor m_templateProcessor;
    ArucoProcessor m_markerProcessor;

    mutable QMutex m_mutex;

private:
    void initialize();

};

class MonocularProcessorThread : public ProcessorThreadBase
{
    Q_OBJECT

public:
    explicit MonocularProcessorThread( QObject *parent = nullptr );

    void processFrame( const Frame &frame, Type type );

    MonocularProcessorResult result() const;

protected:
    Frame m_frame;
    MonocularProcessorResult m_result;

    virtual void run() override;

private:
    void initialize();

};

class StereoProcessorThread : public ProcessorThreadBase
{
    Q_OBJECT

public:
    explicit StereoProcessorThread( QObject *parent = nullptr );

    void processFrame( const StereoFrame &frame, Type type );

    StereoProcessorResult result() const;

protected:
    StereoFrame m_frame;
    StereoProcessorResult m_result;

    virtual void run() override;

private:
    void initialize();

};
