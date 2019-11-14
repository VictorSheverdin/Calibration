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

// FeatureFrame
FeatureProcessor FeatureFrame::m_processor;

FeatureFrame::FeatureFrame()
{
}

FeatureFrame::FeatureFrame( const CvImage &image )
{
    load( image );
}

FeatureFrame::FramePtr FeatureFrame::create()
{
    return FramePtr( new FeatureFrame() );
}

FeatureFrame::FramePtr FeatureFrame::create( const CvImage &image )
{
    return FramePtr( new FeatureFrame( image ) );
}

void FeatureFrame::load( const CvImage &image )
{
    m_processor.extract( image, &m_keyPoints, &m_descriptors );
}

size_t FeatureFrame::pointsCount() const
{
    return m_framePoints.size();
}

bool FeatureFrame::drawKeyPoints( CvImage *target ) const
{
    return drawFeaturePoints( target, m_keyPoints );
}

bool FeatureFrame::drawPoints( CvImage *target ) const
{
    if ( !target )
        return false;

    for ( auto &i : m_framePoints )
        drawFeaturePoint( target, i->point() );

    return true;
}

FeatureFrame::PointPtr FeatureFrame::createFramePoint( const size_t keyPointIndex )
{
    if ( keyPointIndex >= m_keyPoints.size() )
        return PointPtr();

    auto frame = FeaturePoint::create( shared_from_this(), keyPointIndex );

    m_framePoints.push_back( frame );

    return frame;

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
    matchFrames( FeatureFrame::create( leftImage ), FeatureFrame::create( rightImage ) );
}

void StereoFrame::matchFrames( const FeatureFramePtr leftFrame, const FeatureFramePtr rightFrame )
{
    if ( leftFrame && rightFrame ) {

        setFrames( leftFrame, rightFrame );

        std::vector< cv::DMatch > matches;

        m_matcher.match( leftFrame->m_keyPoints, leftFrame->m_descriptors, rightFrame->m_keyPoints, rightFrame->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto leftPoint = leftFrame->createFramePoint( i.queryIdx );
            auto rightPoint = rightFrame->createFramePoint( i.trainIdx );

            createFramePoint( leftPoint, rightPoint );

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

StereoFrame::PointPtr StereoFrame::createFramePoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint )
{
    auto frame = std::shared_ptr< StereoPoint >( new StereoPoint( leftPoint, rightPoint ) );

    m_framePoints.push_back( frame );

    return frame;
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

    if ( leftFeatureFrame && rightFeatureFrame )
        std::cout << leftFeatureFrame->pointsCount() << " " << rightFeatureFrame->pointsCount() << std::endl;

    return stackImages( left, right );
}

CvImage StereoFrame::drawStereoPoints( const CvImage &leftImage, const CvImage &rightImage ) const
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

ConsecutiveFrame::FramePtr ConsecutiveFrame::create()
{
    return FramePtr( new ConsecutiveFrame() );
}

void ConsecutiveFrame::load( const CvImage &image1, const CvImage &image2 )
{
    setFrames( FeatureFrame::create( image1 ), FeatureFrame::create( image2 ) );
}

void ConsecutiveFrame::matchFrames( const FeatureFramePtr frame1, const FeatureFramePtr frame2 )
{
    if ( frame1 && frame2 ) {

        setFrames( frame1, frame2 );

        std::vector< cv::DMatch > matches;

        m_matcher.match( frame1->m_keyPoints, frame1->m_descriptors, frame2->m_keyPoints, frame2->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto point1 = frame1->createFramePoint( i.queryIdx );
            auto point2 = frame2->createFramePoint( i.trainIdx );

            createFramePoint( point1, point2 );

        }

    }

}

CvImage ConsecutiveFrame::drawTrack( const CvImage &image ) const
{
    CvImage ret;

    image.copyTo( ret );

    for ( auto &i : m_framePoints )
        drawLine( &ret, i->m_point1->point(), i->m_point2->point() );

    for ( auto &i : m_framePoints )
        drawFeaturePoint( &ret, i->m_point2->point() );

    return ret;

}

ConsecutiveFrame::PointPtr ConsecutiveFrame::createFramePoint(const MonoPointPtr point1, const MonoPointPtr point2 )
{
    auto frame = PointPtr( new ConsecutivePoint( point1, point2 ) );

    m_framePoints.push_back( frame );

    return frame;

}

}

