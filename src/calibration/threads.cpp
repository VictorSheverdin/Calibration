#include "src/common/precompiled.h"

#include "threads.h"

#include <thread>

ProcessorThreadBase::ProcessorThreadBase( QObject *parent )
    : QThread( parent )
{
    initialize();
}

void ProcessorThreadBase::initialize()
{
    m_type = NONE;
}

const TemplateProcessor &ProcessorThreadBase::templateProcessor() const
{
    return m_templateProcessor;
}

TemplateProcessor &ProcessorThreadBase::templateProcessor()
{
    return m_templateProcessor;
}

const ArucoProcessor &ProcessorThreadBase::markerProcessor() const
{
    return m_markerProcessor;
}

ArucoProcessor &ProcessorThreadBase::markerProcessor()
{
    return m_markerProcessor;
}

// MonocularProcessorThread
MonocularProcessorThread::MonocularProcessorThread( QObject *parent )
    : ProcessorThreadBase( parent )
{
    initialize();
}

void MonocularProcessorThread::initialize()
{
}

void MonocularProcessorThread::processFrame( const Frame &frame, Type type )
{
    m_mutex.lock();
    m_frame = frame;
    m_type = type;
    m_mutex.unlock();

}

MonocularProcessorResult MonocularProcessorThread::result() const
{
    MonocularProcessorResult ret;

    m_mutex.lock();
    ret = m_result;
    m_mutex.unlock();

    return ret;

}

void MonocularProcessorThread::run()
{
    while( true ) {

        if ( m_type == TEMPLATE ) {
            m_type = NONE;

            m_result.sourceFrame = m_frame;

            m_result.exist = m_templateProcessor.processFrame( m_frame, &m_result.preview, &m_result.imagePoints );

            if ( m_result.exist )
                m_templateProcessor.calcCorners( &m_result.worldPoints );

            emit updateSignal();

        }
        else if ( m_type == MARKER ) {
            m_type = NONE;

            m_result.sourceFrame = m_frame;

            ArucoMarkerList list;

            m_result.exist = m_markerProcessor.processFrame( m_frame, &m_result.preview, &list );

            if ( m_result.exist ) {
                m_result.imagePoints = list.points();
                m_result.worldPoints = m_markerProcessor.calcCorners( list );
            }

            emit updateSignal();
        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );

    }

}

// StereoProcessorThread
StereoProcessorThread::StereoProcessorThread( QObject *parent )
    : ProcessorThreadBase( parent )
{
    initialize();
}

void StereoProcessorThread::initialize()
{
}

void StereoProcessorThread::processFrame( const StereoFrame &frame, Type type )
{
    m_mutex.lock();
    m_frame = frame;
    m_type = type;
    m_mutex.unlock();

}

StereoProcessorResult StereoProcessorThread::result() const
{
    StereoProcessorResult ret;

    m_mutex.lock();
    ret = m_result;
    m_mutex.unlock();

    return ret;

}

void StereoProcessorThread::run()
{
    while( true ) {

        if ( m_type == TEMPLATE ) {
            m_type = NONE;

            m_result.sourceFrame = m_frame;

            m_result.leftExist = m_templateProcessor.processFrame( m_frame.leftFrame(), &m_result.leftPreview, &m_result.leftImagePoints );
            m_result.rightExist = m_templateProcessor.processFrame( m_frame.rightFrame(), &m_result.rightPreview, &m_result.rightImagePoints );

            if ( m_result.leftExist && m_result.rightExist )
                m_templateProcessor.calcCorners( &m_result.worldPoints );

            emit updateSignal();

        }
        else if ( m_type == MARKER ) {
            m_type = NONE;

            m_result.sourceFrame = m_frame;

            ArucoMarkerList list;

            m_result.leftExist = m_markerProcessor.processFrame( m_frame.leftFrame(), &m_result.leftPreview, &list );

            if ( m_result.leftExist )
                m_result.leftImagePoints = list.points();

            m_result.rightExist = m_markerProcessor.processFrame( m_frame.rightFrame(), &m_result.rightPreview, &list );

            if ( m_result.rightExist )
                m_result.rightImagePoints = list.points();

            if ( m_result.leftExist && m_result.rightExist )
                m_result.worldPoints = m_markerProcessor.calcCorners( list );

            emit updateSignal();
        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );

    }

}


