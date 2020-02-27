#include "src/common/precompiled.h"

#include "threads.h"

#include <thread>

// MonocularProcessorThread
MonocularProcessorThread::MonocularProcessorThread( QObject *parent )
    : QThread( parent )
{
    initialize();

}

void MonocularProcessorThread::initialize()
{
    m_type = NONE;
}

const TemplateProcessor &MonocularProcessorThread::templateProcessor() const
{
    return m_templateProcessor;
}

TemplateProcessor &MonocularProcessorThread::templateProcessor()
{
    return m_templateProcessor;
}

const ArucoProcessor &MonocularProcessorThread::markerProcessor() const
{
    return m_markerProcessor;
}

ArucoProcessor &MonocularProcessorThread::markerProcessor()
{
    return m_markerProcessor;
}

void MonocularProcessorThread::processFrame( const Frame &frame )
{
    m_mutex.lock();
    m_frame = frame;
    m_mutex.unlock();

}

void MonocularProcessorThread::run()
{
    while( true ) {

        if ( m_type == TEMPLATE ) {
            m_type = NONE;

            m_templateProcessor.processFrame( m_frame, &m_preview, &m_imagePoints );

            emit updateSignal();

        }
        else if ( m_type == MARKER ) {
            m_type = NONE;

            ArucoMarkerList list;

            m_markerProcessor.processFrame( m_frame, &m_preview, &list );

            emit updateSignal();
        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );

    }
}

// StereoProcessorThread
StereoProcessorThread::StereoProcessorThread( QObject *parent )
    : QThread( parent )
{
    initialize();
}

void StereoProcessorThread::initialize()
{
}

void StereoProcessorThread::run()
{
}


