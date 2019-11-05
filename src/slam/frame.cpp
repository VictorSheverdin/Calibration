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

// DoubleFrameBase
DoubleFrameBase::DoubleFrameBase()
{
}

DoubleFrameBase::DoubleFrameBase( const CvImage &image1,  const CvImage &image2 )
{
    load( image1, image2 );
}

void DoubleFrameBase::load( const CvImage &image1,  const CvImage &image2 )
{
    m_frame1 = std::make_shared< MonoFrame >( image1 );
    m_frame2 = std::make_shared< MonoFrame >( image2 );
}

// StereoFrame
StereoFrame::StereoFrame()
{
}

StereoFrame::StereoFrame( const CvImage &leftImage,  const CvImage &rightImage )
    : DoubleFrameBase( leftImage, rightImage )
{
}

void StereoFrame::triangulatePoints()
{
    if ( leftFrame() && rightFrame() ) {

        std::vector< cv::DMatch > matches;

        m_processor.match( leftFrame()->m_keyPoints, leftFrame()->m_descriptors, rightFrame()->m_keyPoints, rightFrame()->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto leftPoint = leftFrame()->createFramePoint( i.queryIdx );
            auto rightPoint = rightFrame()->createFramePoint( i.trainIdx );

            createFramePoint( leftPoint, rightPoint );

        }

    }

}

void StereoFrame::setLeftFrame( const std::shared_ptr<MonoFrame> &value )
{
    m_frame1 = value;
}

void StereoFrame::setRightFrame( const std::shared_ptr<MonoFrame> &value )
{
    m_frame2 = value;
}

std::shared_ptr< MonoFrame > &StereoFrame::leftFrame()
{
    return m_frame1;
}

std::shared_ptr< MonoFrame > &StereoFrame::rightFrame()
{
    return m_frame2;
}

const std::shared_ptr< MonoFrame > &StereoFrame::leftFrame() const
{
    return m_frame1;
}

const std::shared_ptr< MonoFrame > &StereoFrame::rightFrame() const
{
    return m_frame2;
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

    leftFrame()->drawKeyPoints( &left );
    rightFrame()->drawKeyPoints( &right );

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

// ConsecutiveFrame
ConsecutiveFrame::ConsecutiveFrame()
{
}

ConsecutiveFrame::ConsecutiveFrame( const CvImage &leftImage, const CvImage &rightImage )
    : DoubleFrameBase( leftImage, rightImage )
{
}

void ConsecutiveFrame::recoverPose()
{
    if ( m_frame1 && m_frame2 ) {

        std::vector< cv::DMatch > matches;

        m_processor.match( m_frame1->m_keyPoints, m_frame1->m_descriptors, m_frame2->m_keyPoints, m_frame2->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto point1 = m_frame1->createFramePoint( i.queryIdx );
            auto point2 = m_frame2->createFramePoint( i.trainIdx );

            createFramePoint( point1, point2 );

        }

    }
}

CvImage ConsecutiveFrame::drawTrack( const CvImage &image ) const
{
    CvImage ret;

    image.copyTo( ret );

    for ( auto &i : m_framePoints )
        drawLine( &ret, i->m_framePoint1->point(), i->m_framePoint2->point() );

    for ( auto &i : m_framePoints )
        drawFeaturePoint( &ret, i->m_framePoint2->point() );

    return ret;

}

std::shared_ptr< ConsecutiveFramePoint > ConsecutiveFrame::createFramePoint( const std::shared_ptr< MonoFramePoint > &point1, const std::shared_ptr< MonoFramePoint > &point2 )
{
    auto frame = std::shared_ptr< ConsecutiveFramePoint >( new ConsecutiveFramePoint( point1, point2 ) );

    m_framePoints.push_back( frame );

    return frame;

}

}

