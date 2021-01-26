#include "src/common/precompiled.h"

#include "thread.h"

#include "system.h"

// ProcessorThread
ProcessorThread::ProcessorThread( QObject *parent )
    : QThread( parent )
{
    slam2::Parameters parameters;

    _system = slam2::System::create( parameters );

    _system->createMap();
}

void ProcessorThread::process( const StampedStereoImage image )
{
    QMutexLocker lock( &_queueMutex );

    _processQueue.push_back( image );
}

slam2::SystemPtr ProcessorThread::system() const
{
    return _system;
}

CvImage ProcessorThread::pointsImage() const
{
    return CvImage();
}

CvImage ProcessorThread::tracksImage() const
{
    return CvImage();
}

CvImage ProcessorThread::stereoImage() const
{
    return CvImage();
}

void ProcessorThread::run()
{
    while( !isInterruptionRequested() )
    {
        StampedStereoImage frame;

        _queueMutex.lock();

        if ( !_processQueue.empty() ) {
            frame = _processQueue.front();
            _processQueue.pop_front();
        }

        _queueMutex.unlock();

        if ( !frame.empty() ) {

            _system->track( frame );

        }

        std::this_thread::sleep_for( std::chrono::microseconds( 1 ) );

    }
}

