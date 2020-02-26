#include "src/common/precompiled.h"

#include "threads.h"

// MonocularProcessorThread
MonocularProcessorThread::MonocularProcessorThread( const TemplateProcessor &processor, QObject *parent )
    : QThread( parent )
{
    initialize();

    setProcessor( processor );

}

void MonocularProcessorThread::initialize()
{
}

void MonocularProcessorThread::setProcessor( const TemplateProcessor &processor )
{
    m_processor = processor;
}

const TemplateProcessor &MonocularProcessorThread::processor() const
{
    return m_processor;
}

TemplateProcessor &MonocularProcessorThread::processor()
{
    return m_processor;
}

void MonocularProcessorThread::addFrame( const Frame &frame )
{
    m_framesMutex.lock();
    m_framesQueue.push( frame );
    m_framesMutex.unlock();

}

void MonocularProcessorThread::run()
{
    if ( !m_framesQueue.empty() ) {
        /*CvImage procFrame;
        std::vector< cv::Point2f > previewPoints;
        m_processor.processPreview( m_frame, &procFrame, &previewPoints );*/

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


