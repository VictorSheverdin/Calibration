#pragma once

#include <QThread>
#include <QMutex>

#include "src/common/templateprocessor.h"

class MonocularProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit MonocularProcessorThread( const TemplateProcessor &processor, QObject *parent = nullptr );

    void setProcessor( const TemplateProcessor &processor );

    const TemplateProcessor &processor() const;
    TemplateProcessor &processor();

    void addFrame( const Frame &frame );

protected:
    TemplateProcessor m_processor;

    LimitedQueue< Frame > m_framesQueue;
    QMutex m_framesMutex;

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
