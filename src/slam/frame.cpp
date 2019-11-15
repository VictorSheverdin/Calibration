#include "src/common/precompiled.h"

#include "frame.h"

#include "src/common/functions.h"

namespace slam {

// FrameBase
FrameBase::FrameBase()
{
}

// MonoFrame
MonoFrame::MonoFrame()
{
}

std::vector< MonoFrame::PointPtr > &MonoFrame::points()
{
    return m_points;
}

const std::vector< MonoFrame::PointPtr > &MonoFrame::points() const
{
    return m_points;
}

MonoFrame::PointPtr MonoFrame::point( const size_t index ) const
{
    return m_points[ index ];
}

size_t MonoFrame::pointsCount() const
{
    return m_points.size();
}

size_t MonoFrame::tracksCount() const
{
    size_t count = 0;

    for ( auto &i : m_points )
        if ( i )
            if ( i->prevPoint() || i->nextPoint() )
                ++count;

    return count;

}

bool MonoFrame::drawPoints( CvImage *target ) const
{
    if ( !target )
        return false;

    for ( auto &i : m_points )
        drawFeaturePoint( target, i->point() );

    return true;
}

// FeatureFrame
FeatureProcessor FeatureFrame::m_processor;

FeatureFrame::FeatureFrame()
{
}

FeatureFrame::FramePtr FeatureFrame::create()
{
    return FramePtr( new FeatureFrame() );
}

void FeatureFrame::load( const CvImage &image )
{
    m_processor.extract( image, &m_keyPoints, &m_descriptors );

    for ( size_t i = 0; i < m_keyPoints.size(); ++i )
        createFramePoint( i );
}

bool FeatureFrame::drawKeyPoints( CvImage *target ) const
{
    return drawFeaturePoints( target, m_keyPoints );
}

FeatureFrame::PointPtr FeatureFrame::createFramePoint( const size_t keyPointIndex )
{
    if ( keyPointIndex >= m_keyPoints.size() )
        return PointPtr();

    auto point = FeaturePoint::create( shared_from_this(), keyPointIndex );

    m_points.push_back( point );

    return point;

}

// DoubleFrameBase
FeatureMatcher DoubleFrameBase::m_matcher;

DoubleFrameBase::DoubleFrameBase()
{
}

void DoubleFrameBase::setFrames( const MonoFramePtr frame1, const MonoFramePtr frame2 )
{
    m_frame1 = frame1;
    m_frame2 = frame2;
}

DoubleFrameBase::MonoFramePtr DoubleFrameBase::frame1() const
{
    return m_frame1;
}

DoubleFrameBase::MonoFramePtr DoubleFrameBase::frame2() const
{
    return m_frame2;
}

// StereoFrame
StereoFrame::StereoFrame()
{
}

StereoFrame::FramePtr StereoFrame::create()
{
    return FramePtr( new StereoFrame() );
}

void StereoFrame::load( const CvImage &leftImage, const CvImage &rightImage )
{
    auto leftFrame = FeatureFrame::create();
    leftFrame->load( leftImage );

    auto rightFrame = FeatureFrame::create();
    rightFrame->load( rightImage );

    matchFrames( leftFrame, rightFrame );
}

void StereoFrame::matchFrames( const FeatureFramePtr leftFrame, const FeatureFramePtr rightFrame )
{
    if ( leftFrame && rightFrame ) {

        setFrames( leftFrame, rightFrame );

        std::vector< cv::DMatch > matches;

        m_matcher.match( leftFrame->m_keyPoints, leftFrame->m_descriptors, rightFrame->m_keyPoints, rightFrame->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto leftPoint = leftFrame->point( i.queryIdx );
            auto rightPoint = rightFrame->point( i.trainIdx );

            leftPoint->setStereoPoint( rightPoint );
            rightPoint->setStereoPoint( leftPoint );

        }

    }

}

DoubleFrameBase::MonoFramePtr StereoFrame::leftFrame() const
{
    return frame1();
}

DoubleFrameBase::MonoFramePtr StereoFrame::rightFrame() const
{
    return frame2();
}

CvImage StereoFrame::drawKeyPoints( const CvImage &leftImage, const CvImage &rightImage ) const
{
    CvImage left;
    leftImage.copyTo( left );
    CvImage right;
    rightImage.copyTo( right );

    auto leftFeatureFrame = dynamic_cast< FeatureFrame * >( leftFrame().get() );
    auto rightFeatureFrame = dynamic_cast< FeatureFrame * >( rightFrame().get() );

    if ( leftFeatureFrame )
        leftFeatureFrame->drawKeyPoints( &left );

    if ( rightFeatureFrame )
        rightFeatureFrame->drawKeyPoints( &right );

    return stackImages( left, right );
}

CvImage StereoFrame::drawStereoPoints( const CvImage &leftImage, const CvImage &rightImage ) const
{
    CvImage ret;

    ret = stackImages( leftImage, rightImage );

    if ( leftFrame() ) {

        for ( auto &i : leftFrame()->points() ) {

            if ( i && i->stereoPoint() ) {

                auto rightPoint = i->stereoPoint()->point();
                rightPoint.x += leftImage.width();

                drawLine( &ret, i->point(), rightPoint );

            }

        }

        for ( auto &i : leftFrame()->points() ) {

            if ( i && i->stereoPoint() ) {

                auto rightPoint = i->stereoPoint()->point();
                rightPoint.x += leftImage.width();

                drawFeaturePoint( &ret, i->point() );
                drawFeaturePoint( &ret, rightPoint );

            }

        }

    }

    return ret;

}

// AdjacentFrame
AdjacentFrame::AdjacentFrame()
{
}

AdjacentFrame::FramePtr AdjacentFrame::create()
{
    return FramePtr( new AdjacentFrame() );
}

void AdjacentFrame::load( const CvImage &image1, const CvImage &image2 )
{
    auto frame1 = FeatureFrame::create();
    frame1->load( image1 );

    auto frame2 = FeatureFrame::create();
    frame2->load( image2 );

    setFrames( frame1, frame2 );

}

void AdjacentFrame::matchFrames( const FeatureFramePtr frame1, const FeatureFramePtr frame2 )
{
    if ( frame1 && frame2 ) {

        setFrames( frame1, frame2 );

        std::vector< cv::DMatch > matches;

        m_matcher.match( frame1->m_keyPoints, frame1->m_descriptors, frame2->m_keyPoints, frame2->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto point1 = frame1->point( i.queryIdx );
            auto point2 = frame2->point( i.trainIdx );

            point1->setNextPoint( point2 );
            point2->setPrevPoint( point1 );

        }

    }

}

CvImage AdjacentFrame::drawTrack( const CvImage &image ) const
{
    CvImage ret;

    image.copyTo( ret );

    if ( m_frame2 ) {

        for ( auto &i : m_frame2->points() )
            if ( i )
                i->drawTrack( &ret );

        for ( auto &i : m_frame2->points() )
            if ( i && i->prevPoint() )
                drawFeaturePoint( &ret, i->point(), 2 );

    }

    return ret;

}

}

