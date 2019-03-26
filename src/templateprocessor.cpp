#include "precompiled.h"

#include "templateprocessor.h"

// ProcessorData
ProcessorData::ProcessorData()
{
    initialize();
}

void ProcessorData::initialize()
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

void TemplateProcessor::setAdaptiveThreshold( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_ADAPTIVE_THRESH;
    else
        m_flags &= !cv::CALIB_CB_ADAPTIVE_THRESH;

}

void TemplateProcessor::setNormalizeImage( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_NORMALIZE_IMAGE;
    else
        m_flags &= !cv::CALIB_CB_NORMALIZE_IMAGE;

}

void TemplateProcessor::setFilterQuads( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_FILTER_QUADS;
    else
        m_flags &= !cv::CALIB_CB_FILTER_QUADS;

}

void TemplateProcessor::setFastCheck( const bool value )
{
    if (value)
        m_flags |= cv::CALIB_CB_FAST_CHECK;
    else
        m_flags &= !cv::CALIB_CB_FAST_CHECK;

}

bool TemplateProcessor::process( const CvImage &frame, CvImage *procFrame, std::vector<cv::Point2f> *points )
{
    auto extent = std::max( frame.width(), frame.height() );

    if ( m_resizeFlag && extent > m_frameMaximumSize ) {
        double scaleFactor = static_cast<double>( m_frameMaximumSize ) / extent;
        cv::resize( frame, *procFrame, cv::Size(), scaleFactor, scaleFactor, cv::INTER_LANCZOS4 );
    }
    else
        frame.copyTo( *procFrame );

    auto ret = cv::findChessboardCorners( *procFrame, m_count, *points, m_flags ) ;

    if (ret)
        cv::drawChessboardCorners( *procFrame, m_count, *points, true );

    return ret;

}
