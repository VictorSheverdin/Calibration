#include "precompiled.h"

#include "markerprocessor.h"

#include "functions.h"

// ArucoMarker
ArucoMarker::ArucoMarker()
{
}

ArucoMarker::ArucoMarker( const int id, const std::vector< cv::Point2f > &corners )
{
    set( id, corners ) ;
}

void ArucoMarker::setId( const int value )
{
    m_id = value;
}

int ArucoMarker::id() const
{
    return m_id;
}

void ArucoMarker::setCorners( const std::vector< cv::Point2f > &value )
{
    m_corners = value;
}

const std::vector< cv::Point2f > &ArucoMarker::corners() const
{
    return m_corners;
}

void ArucoMarker::set( const int id, const std::vector< cv::Point2f > &corners )
{
    setId( id );
    setCorners( corners );
}

// ArucoProcessor
ArucoProcessor::ArucoProcessor()
{
    initialize();
}

void ArucoProcessor::initialize()
{
    m_dictionary = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_6X6_50 ) ;
}

void ArucoProcessor::setResizeFlag( const bool value )
{
    m_resizeFlag = value;
}

void ArucoProcessor::setFrameMaximumSize( const unsigned int value )
{
    m_frameMaximumSize = value;
}

bool ArucoProcessor::resizeFlag() const
{
    return m_resizeFlag;
}

unsigned int ArucoProcessor::frameMaximumFlag() const
{
    return m_frameMaximumSize;
}

ArucoMarkerList ArucoProcessor::detectMarkers( const CvImage &image )
{
    ArucoMarkerList ret;

    std::vector< int > markerIds;
    std::vector< std::vector< cv::Point2f > > markerCorners;

    cv::aruco::detectMarkers( image, m_dictionary, markerCorners, markerIds );

    if ( markerCorners.size() != markerIds.size() )
        return ret;

    for ( size_t i = 0; i < markerCorners.size(); ++i )
        ret.push_back( ArucoMarker( markerIds[ i ], markerCorners[ i ] ) );

    return ret;

}

bool ArucoProcessor::processFrame( const Frame &frame, CvImage *view, ArucoMarkerList *markers )
{
    bool ret = false;

    if ( !frame.empty() ) {
        auto markersList = detectMarkers( frame );

        if ( !markersList.empty() )
            ret = true;

        if (view) {
            frame.copyTo( *view );

            if (ret && view)
                drawMarkers( view, markersList );

        }

        if ( markers )
            *markers = markersList;

    }

    return ret;
}

bool ArucoProcessor::processPreview( const Frame &frame, CvImage *preview )
{
    bool ret = false;

    if ( !frame.empty() ) {

        auto extent = std::max( frame.width(), frame.height() );

        cv::Mat sourceFrame;

        if ( m_resizeFlag && extent > m_frameMaximumSize )
            sourceFrame = resizeTo( frame, m_frameMaximumSize );
        else
            frame.copyTo( sourceFrame );

        ret = processFrame( sourceFrame, preview, nullptr );

    }

    return ret;

}

