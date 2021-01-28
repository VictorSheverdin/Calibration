#include "src/common/precompiled.h"

#include "thread.h"

#include "system.h"

#include "src/common/tictoc.h"

// ProcessorThread
ProcessorThread::ProcessorThread( const slam2::Parameters &parameters, QObject *parent )
    : QThread( parent )
{
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
    _resultMutex.lock();
    auto ret = _pointsImage;
    _resultMutex.unlock();

    return ret;
}

CvImage ProcessorThread::tracksImage() const
{
    _resultMutex.lock();
    auto ret = _tracksImage;
    _resultMutex.unlock();

    return ret;
}

CvImage ProcessorThread::stereoImage() const
{
    _resultMutex.lock();
    auto ret = _stereoImage;
    _resultMutex.unlock();

    return ret;
}

std::vector< ColorPoint3d > ProcessorThread::sparseCloud() const
{
    _resultMutex.lock();
    auto ret = _sparseCloud;
    _resultMutex.unlock();

    return ret;
}

void ProcessorThread::run()
{
    while( !isInterruptionRequested() ) {
        processNext();
        std::this_thread::sleep_for( std::chrono::microseconds( 1 ) );

    }

}

void ProcessorThread::processNext()
{
    StampedStereoImage frame;

    _queueMutex.lock();

    if ( !_processQueue.empty() ) {
        frame = _processQueue.front();
        _processQueue.pop_front();
    }

    _queueMutex.unlock();

    if ( !frame.empty() ) {

        TicToc timer;

        _system->track( frame );

        timer.report();

        _resultMutex.lock();

        _pointsImage = _system->pointsImage();
        _tracksImage = _system->tracksImage();
        _stereoImage = _system->stereoImage();
        _sparseCloud = _system->sparseCloud();

        _resultMutex.unlock();

    }

}


