#include "src/common/precompiled.h"

#include "templateprocessor.h"

#include "src/common/functions.h"

// ProcessorData
ProcessorState::ProcessorState()
{
    initialize();
}

void ProcessorState::initialize()
{
}

// TemplateProcessor
TemplateProcessor::TemplateProcessor()
{
    initialize();
}

void TemplateProcessor::initialize()
{
    m_templateType = CHECKERBOARD;

    m_count = cv::Size( 3, 3 );

    m_size = 1.0;

    m_resizeFlag = false;

    m_subPixFlag = true;
    m_subPixWinSize = cv::Size( 11, 11 );
    m_subPixZeroZone = cv::Size( -1, -1 );

    m_frameMaximumSize = 500;

    m_flags = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

}

void TemplateProcessor::setType( const Type type )
{
    m_templateType = type;
}

void TemplateProcessor::setCount( const cv::Size &count )
{
    m_count = count;
}

void TemplateProcessor::setSize( const double value )
{
    m_size = value;
}

void TemplateProcessor::setResizeFlag( const bool value )
{
    m_resizeFlag = value;
}

void TemplateProcessor::setFrameMaximumSize( const unsigned int value )
{
    m_frameMaximumSize = value;
}

TemplateProcessor::Type TemplateProcessor::type() const
{
    return m_templateType;
}

const cv::Size &TemplateProcessor::count() const
{
    return m_count;
}

double TemplateProcessor::size() const
{
    return m_size;
}

bool TemplateProcessor::resizeFlag() const
{
    return m_resizeFlag;
}

unsigned int TemplateProcessor::frameMaximumFlag() const
{
    return m_frameMaximumSize;
}

bool TemplateProcessor::adaptiveThreshold() const
{
    return m_flags & cv::CALIB_CB_ADAPTIVE_THRESH;
}

bool TemplateProcessor::normalizeImage() const
{
    return m_flags & cv::CALIB_CB_NORMALIZE_IMAGE;
}

bool TemplateProcessor::filterQuads() const
{
    return m_flags & cv::CALIB_CB_FILTER_QUADS;
}

bool TemplateProcessor::fastCheck() const
{
    return m_flags & cv::CALIB_CB_FAST_CHECK;
}

void TemplateProcessor::setAdaptiveThreshold( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
    else
        m_flags &= ~cv::CALIB_CB_ADAPTIVE_THRESH;

}

void TemplateProcessor::setNormalizeImage( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
    else
        m_flags &= ~cv::CALIB_CB_NORMALIZE_IMAGE;

}

void TemplateProcessor::setFilterQuads( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_FILTER_QUADS;
    else
        m_flags &= ~cv::CALIB_CB_FILTER_QUADS;

}

void TemplateProcessor::setFastCheck( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_FAST_CHECK;
    else
        m_flags &= ~cv::CALIB_CB_FAST_CHECK;

}

bool TemplateProcessor::processFrame(const Frame &frame, CvImage *view, std::vector< cv::Point2f > *points )
{
    bool ret = false;

    if ( !frame.empty() ) {
        std::vector< cv::Point2f > pointsVec;

        ret = findPoints( frame, &pointsVec );

        if (view) {
            frame.copyTo( *view );

            if (ret && view)
                cv::drawChessboardCorners( *view, m_count, pointsVec, true );

        }

        if (points)
            *points = pointsVec;

    }

    return ret;

}

bool TemplateProcessor::processPreview(const Frame &frame, CvImage *preview, std::vector< cv::Point2f > *points )
{
    bool ret = false;

    if ( !frame.empty() ) {

        auto extent = std::max( frame.width(), frame.height() );

        cv::Mat sourceFrame;

        if ( m_resizeFlag && extent > m_frameMaximumSize )
            sourceFrame = resizeTo( frame, m_frameMaximumSize );
        else
            frame.copyTo( sourceFrame );

        ret = processFrame( sourceFrame, preview, points );

    }

    return ret;

}

bool TemplateProcessor::calcChessboardCorners( std::vector< cv::Point3f > *corners )
{
    corners->clear();

    switch( m_templateType )
    {
        case CHECKERBOARD:
        case CIRCLES:
            for( int i = 0; i < m_count.height; i++ )
                for( int j = 0; j < m_count.width; j++ )
                    corners->push_back( cv::Point3f( j * m_size, i * m_size, 0 ) );
            return true;

        case ASYM_CIRCLES:
            for( int i = 0; i < m_count.height; i++ )
                for( int j = 0; j < m_count.width; j++ )
                    corners->push_back( cv::Point3f( ( 2*j + i % 2 ) * m_size, i * m_size, 0 ) );
            return true;
        case ARUCO_MARKERS:
            break;

    }

    return false;

}

bool TemplateProcessor::findPoints(const CvImage &frame, std::vector<cv::Point2f> *points )
{
    bool ret = false;

    if ( m_templateType == CHECKERBOARD ) {
        ret = cv::findChessboardCorners( frame, m_count, *points, m_flags ) ;

        if ( ret && !points->empty() && m_subPixFlag ) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix( gray, *points, m_subPixWinSize, m_subPixZeroZone, cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1 ) );

        }

    }
    else if ( m_templateType == CIRCLES )
        ret = cv::findCirclesGrid( frame, m_count, *points, cv::CALIB_CB_SYMMETRIC_GRID );
    else if ( m_templateType == ASYM_CIRCLES )
        ret = cv::findCirclesGrid( frame, m_count, *points, cv::CALIB_CB_ASYMMETRIC_GRID );

    return ret;

}

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


