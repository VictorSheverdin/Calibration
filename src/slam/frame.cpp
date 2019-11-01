#include "src/common/precompiled.h"

#include "frame.h"

#include "src/common/functions.h"

namespace slam {

// FrameBase
FeatureProcessor FrameBase::m_processor;

FrameBase::FrameBase()
{
}

// MonoFrame
MonoFrame::MonoFrame()
{
}

MonoFrame::MonoFrame( const CvImage &image )
{
    load( image );
}

void MonoFrame::load( const CvImage &image )
{
    m_processor.extract( image, &m_keyPoints, &m_descriptors );
}

bool MonoFrame::drawKeyPoints( CvImage *target ) const
{
    return drawFeaturePoints( target, m_keyPoints );
}

bool MonoFrame::drawFramePoints( CvImage *target ) const
{
    if ( !target )
        return false;

    for ( auto &i : m_framePoints )
        drawFeaturePoint( target, i->point() );

    return true;
}

std::shared_ptr<MonoFramePoint> MonoFrame::createFramePoint( const size_t keyPointIndex )
{
    if ( keyPointIndex >= m_keyPoints.size() )
        return std::shared_ptr<MonoFramePoint>();

    auto frame = std::shared_ptr< MonoFramePoint >( new MonoFramePoint( this, keyPointIndex ) );

    m_framePoints.push_back( frame );

    return frame;

}

// StereoFrame
StereoFrame::StereoFrame()
{
}

StereoFrame::StereoFrame( const CvImage &leftImage,  const CvImage &rightImage )
{
    load( leftImage, rightImage );
}

void StereoFrame::load( const CvImage &leftImage,  const CvImage &rightImage )
{
    m_leftFrame.load( leftImage );
    m_rightFrame.load( rightImage );
}

void StereoFrame::triangulatePoints()
{
    std::vector< cv::DMatch > matches;

    m_processor.match( m_leftFrame.m_keyPoints ,m_leftFrame.m_descriptors, m_rightFrame.m_keyPoints, m_rightFrame.m_descriptors, &matches );

    for ( auto &i : matches ) {
        auto leftPoint = m_leftFrame.createFramePoint( i.queryIdx );
        auto rightPoint = m_rightFrame.createFramePoint( i.trainIdx );

        createFramePoint( leftPoint, rightPoint );

    }

}

std::shared_ptr< StereoFramePoint > StereoFrame::createFramePoint( const std::shared_ptr< MonoFramePoint > &leftPoint, const std::shared_ptr< MonoFramePoint > &rightPoint )
{
    auto frame = std::shared_ptr< StereoFramePoint >( new StereoFramePoint( leftPoint, rightPoint ) );

    m_framePoints.push_back( frame );

    return frame;
}

CvImage StereoFrame::drawKeyPoints( const CvImage &leftImage, const CvImage &rightImage ) const
{
    CvImage left;
    leftImage.copyTo( left );
    CvImage right;
    rightImage.copyTo( right );

    m_leftFrame.drawKeyPoints( &left );
    m_rightFrame.drawKeyPoints( &right );

    return stackImages( left, right );
}

CvImage StereoFrame::drawStereoPoints(const CvImage &leftImage, const CvImage &rightImage ) const
{
    CvImage ret;

    ret = stackImages( leftImage, rightImage );

    for ( auto &i : m_framePoints ) {
        auto rightPoint = i->rightPoint();
        rightPoint.x += leftImage.width();

        drawLine( &ret, i->leftPoint(), rightPoint );

    }

    for ( auto &i : m_framePoints ) {
        auto rightPoint = i->rightPoint();
        rightPoint.x += leftImage.width();

        drawFeaturePoint( &ret, i->leftPoint() );
        drawFeaturePoint( &ret, rightPoint );

    }

    return ret;

}

}

