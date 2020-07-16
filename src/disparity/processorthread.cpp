#include "src/common/precompiled.h"

#include "processorthread.h"

#include "src/common/functions.h"

// ProcessorThread
ProcessorThread::ProcessorThread( QObject *parent )
    : QThread( parent )
{
    initialize();
}

void ProcessorThread::initialize()
{
}

bool ProcessorThread::process( const StampedStereoImage &frame )
{
    bool res = false;

    if ( m_processMutex.tryLock() ) {
        if ( !isRunning() ) {
            m_frame = frame;
            start();
            res = true;
        }

        m_processMutex.unlock();
    }

    return res;

}

void ProcessorThread::setProcessor( const std::shared_ptr< StereoResultProcessor > processor )
{
    m_processor = processor;
}

StereoResult ProcessorThread::result()
{
    StereoResult ret;

    m_resultMutex.lock();
    ret = m_result;
    m_resultMutex.unlock();

    return ret;
}

void ProcessorThread::run()
{
    if ( !m_frame.empty() ) {
        auto result = m_processor->process( m_frame );
        m_resultMutex.lock();
        m_result = result;
        m_resultMutex.unlock();

        emit frameProcessed();

    }

}
