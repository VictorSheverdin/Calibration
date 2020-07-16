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

void MonocularProcessorThread::processFrame( const StampedImage &frame, Type type )
{
    m_mutex.lock();
    m_frame = frame;
    m_type = type;
    m_mutex.unlock();

}

MonocularProcessorResult MonocularProcessorThread::calculate( const StampedImage &frame , const Type type ) const
{
    MonocularProcessorResult ret;

    if ( type == TEMPLATE ) {

        ret.sourceFrame = frame;

        ret.exist = m_templateProcessor.processFrame( frame, &ret.preview, &ret.imagePoints );

        if ( ret.exist )
            m_templateProcessor.calcCorners( &ret.worldPoints );

    }
    else if ( type == MARKER ) {

        ret.sourceFrame = frame;

        ArucoMarkerList list;

        ret.exist = m_markerProcessor.processFrame( frame, &ret.preview, &list );

        if ( ret.exist ) {
            ret.imagePoints = list.centerPoints();
            ret.worldPoints = m_markerProcessor.calcCentroids( list );

        }

    }

    return ret;
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

        if ( m_type != NONE ) {

            m_result = calculate( m_frame, m_type );

            m_type = NONE;

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

void StereoProcessorThread::processFrame( const StampedStereoImage &frame, Type type )
{
    m_mutex.lock();
    m_frame = frame;
    m_type = type;
    m_mutex.unlock();

}

StereoProcessorResult StereoProcessorThread::calculate( const StampedStereoImage &frame, const Type type ) const
{
    StereoProcessorResult ret;

    if ( type == TEMPLATE ) {

        ret.sourceFrame = frame;

        ret.leftExist = m_templateProcessor.processFrame( frame.leftImage(), &ret.leftPreview, &ret.leftImagePoints );
        ret.rightExist = m_templateProcessor.processFrame( frame.rightImage(), &ret.rightPreview, &ret.rightImagePoints );

        if ( ret.leftExist && ret.rightExist )
            m_templateProcessor.calcCorners( &ret.worldPoints );

    }
    else if ( type == MARKER ) {

        ret.sourceFrame = frame;

        ArucoMarkerList leftList;
        ArucoMarkerList rightList;

        ret.leftExist = m_markerProcessor.processFrame( frame.leftImage(), &ret.leftPreview, &leftList );
        ret.rightExist = m_markerProcessor.processFrame( frame.rightImage(), &ret.rightPreview, &rightList );

        if ( ret.leftExist && ret.rightExist ) {

            fit( &leftList, &rightList );

            ret.leftImagePoints = leftList.centerPoints();
            ret.rightImagePoints = rightList.centerPoints();

            ret.worldPoints = m_markerProcessor.calcCentroids( leftList );

        }

    }

    return ret;
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

        if ( m_type != NONE ) {

            m_result = calculate( m_frame, m_type );

            m_type = NONE;

            emit updateSignal();

        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );

    }

}


