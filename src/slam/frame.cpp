#include "src/common/precompiled.h"

#include "frame.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

#include "world.h"

#include <opencv2/sfm.hpp>

namespace slam {

// FrameBase
FrameBase::FrameBase()
{
}

// MonoFrame
MonoFrame::MonoFrame()
{
    initialize();
}

void MonoFrame::initialize()
{
    setCameraMatrix( cv::Mat::eye( 3, 3, CV_64F ) );

    setRotation( cv::Mat::eye( 3, 3, CV_64F ) );
    setTranslation( cv::Mat::zeros( 3, 1, CV_64F ) );
}

std::vector< MonoFrame::PointPtr > &MonoFrame::framePoints()
{
    return m_points;
}

const std::vector< MonoFrame::PointPtr > &MonoFrame::framePoints() const
{
    return m_points;
}

std::vector< cv::Point2f > MonoFrame::points() const
{
    std::vector< cv::Point2f > ret;

    for ( auto &i : m_points )
        if ( i )
            ret.push_back( i->point() );

    return ret;

}

std::vector< StereoPoint > MonoFrame::stereoPoints() const
{
    std::vector< StereoPoint > ret;

    for ( auto &i : m_points ) {
        if ( i && i->stereoPoint() )
            ret.push_back( StereoPoint( i, i->stereoPoint() ));
    }

    return ret;
}

std::vector< AdjacentPoint > MonoFrame::adjacentPoints() const
{
    std::vector< AdjacentPoint > ret;

    for ( auto &i : m_points ) {
        if ( i && i->nextPoint() )
            ret.push_back( AdjacentPoint( i, i->nextPoint() ));
    }

    return ret;
}

MonoFrame::PointPtr MonoFrame::framePoint( const size_t index ) const
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

    auto points = this->points();

    for ( auto &i : points )
        drawFeaturePoint( target, i );

    return true;
}

void MonoFrame::setCameraMatrix( const cv::Mat &value )
{
    m_projectionMatrix = cv::Mat();

    m_cameraMatrix = value;
}

const cv::Mat &MonoFrame::cameraMatrix() const
{
    return m_cameraMatrix;
}

void MonoFrame::setRotation( const cv::Mat &value )
{
    m_projectionMatrix = cv::Mat();

    m_r = value;
}

const cv::Mat &MonoFrame::rotation() const
{
    return m_r;
}

void MonoFrame::setTranslation( const cv::Mat &value )
{
    m_projectionMatrix = cv::Mat();

    m_t = value;
}

const cv::Mat &MonoFrame::translation() const
{
    return m_t;
}

void MonoFrame::setProjectionMatrix( const cv::Mat &value )
{
    m_projectionMatrix = value;

    cv::Mat r;
    cv::Mat t;

    cv::decomposeProjectionMatrix( m_projectionMatrix, m_cameraMatrix, r, t );

    m_t.at< double >( 0, 0 ) = - t.at< double >( 0, 0 ) / t.at< double >( 3, 0 );
    m_t.at< double >( 1, 0 ) = - t.at< double >( 1, 0 ) / t.at< double >( 3, 0 );
    m_t.at< double >( 2, 0 ) = - t.at< double >( 1, 0 ) / t.at< double >( 3, 0 );

    m_r = r.t();

}

const cv::Mat &MonoFrame::projectionMatrix() const
{
    if ( m_projectionMatrix.empty() ) {
        auto extrinsicMatrix = cv::Mat( 3, 4, CV_64F );
        m_r.copyTo( extrinsicMatrix.rowRange( 0, 3 ).colRange( 0, 3 ) );
        m_t.copyTo( extrinsicMatrix.rowRange( 0, 3 ).col( 3 ) );
        m_projectionMatrix = m_cameraMatrix * extrinsicMatrix;

    }

    return m_projectionMatrix;
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

    for ( size_t i = 0; i < m_keyPoints.size(); ++i ) {
        auto color = image.at< cv::Vec3b >( m_keyPoints[ i ].pt );
        createFramePoint( i, cv::Scalar( color[0], color[1], color[2], 255 ) );

    }
}

bool FeatureFrame::drawKeyPoints( CvImage *target ) const
{
    return drawFeaturePoints( target, m_keyPoints );
}

FeatureFrame::PointPtr FeatureFrame::createFramePoint( const size_t keyPointIndex , const cv::Scalar &color )
{
    if ( keyPointIndex >= m_keyPoints.size() )
        return PointPtr();

    auto point = FeaturePoint::create( shared_from_this(), keyPointIndex, color );

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

    setFrames( leftFrame, rightFrame );

}

void StereoFrame::matchFrames()
{
    auto leftFrame = std::dynamic_pointer_cast< FeatureFrame >( this->leftFrame() );
    auto rightFrame = std::dynamic_pointer_cast< FeatureFrame >( this->rightFrame() );

    if ( leftFrame && rightFrame ) {

        std::vector< cv::DMatch > matches;

        m_matcher.match( leftFrame->m_keyPoints, leftFrame->m_descriptors, rightFrame->m_keyPoints, rightFrame->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto leftPoint = leftFrame->framePoint( i.queryIdx );
            auto rightPoint = rightFrame->framePoint( i.trainIdx );

            double lenght = cv::norm( leftPoint->point() - rightPoint->point() );

            if ( lenght >= m_minLenght ) {
                leftPoint->setStereoPoint( rightPoint );
                rightPoint->setStereoPoint( leftPoint );

            }

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

std::vector< StereoPoint > StereoFrame::stereoPoints() const
{
    auto leftFrame = this->leftFrame();

    if (leftFrame)
        return leftFrame->stereoPoints();
    else
        return std::vector< StereoPoint >();

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

        for ( auto &i : leftFrame()->framePoints() ) {

            if ( i && i->stereoPoint() ) {

                auto rightPoint = i->stereoPoint()->point();
                rightPoint.x += leftImage.width();

                drawLine( &ret, i->point(), rightPoint );

            }

        }

        for ( auto &i : leftFrame()->framePoints() ) {

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

// WorldStereoFrame
WorldStereoFrame::WorldStereoFrame( const WorldPtr &parentWorld )
    : m_parentWorld( parentWorld )
{
}

WorldStereoFrame::FramePtr WorldStereoFrame::create( const WorldPtr &parentWorld )
{
    return FramePtr( new WorldStereoFrame( parentWorld ) );
}

const WorldStereoFrame::WorldPtr WorldStereoFrame::parentWorld() const
{
    return m_parentWorld.lock();
}

void WorldStereoFrame::triangulatePoints()
{
    auto world = parentWorld();

    if ( world ) {

        cv::Mat homogeneousPoints3d;

        auto stereoPoints = this->stereoPoints();

        cv::Mat_< float > leftPoints( 2, stereoPoints.size() ),
                           rightPoints( 2, stereoPoints.size() );

        for ( size_t i = 0; i < stereoPoints.size(); ++i ) {
            leftPoints.row(0).col(i) = stereoPoints[i].leftPoint().x;
            leftPoints.row(1).col(i) = stereoPoints[i].leftPoint().y;
            rightPoints.row(0).col(i) = stereoPoints[i].rightPoint().x;
            rightPoints.row(1).col(i) = stereoPoints[i].rightPoint().y;

        }

        cv::triangulatePoints( leftFrame()->projectionMatrix(), rightFrame()->projectionMatrix(), leftPoints, rightPoints, homogeneousPoints3d );

        for ( size_t i = 0; i < stereoPoints.size(); ++i ) {

            auto w = homogeneousPoints3d.at< float >( 3, i );

            auto x = homogeneousPoints3d.at< float >( 0, i ) / w;
            auto y = homogeneousPoints3d.at< float >( 1, i ) / w;
            auto z = homogeneousPoints3d.at< float >( 2, i ) / w;

            auto pt = cv::Vec3f( x, y, z );

            if ( std::abs( w ) > FLOAT_EPS ) {

                if ( !stereoPoints[i].leftFramePoint()->worldPoint() && !stereoPoints[i].rightFramePoint()->worldPoint() ) {
                    auto worldPoint = world->createWorldPoint( pt, stereoPoints[i].leftFramePoint()->color() );
                    stereoPoints[i].leftFramePoint()->setWorldPoint( worldPoint );
                    stereoPoints[i].rightFramePoint()->setWorldPoint( worldPoint );

                }
                else {
                    if ( !stereoPoints[i].leftFramePoint()->worldPoint() )
                        stereoPoints[i].leftFramePoint()->setWorldPoint( stereoPoints[i].rightFramePoint()->worldPoint() );
                    else if ( !stereoPoints[i].rightFramePoint()->worldPoint() )
                        stereoPoints[i].rightFramePoint()->setWorldPoint( stereoPoints[i].leftFramePoint()->worldPoint() );

                    stereoPoints[i].leftFramePoint()->worldPoint()->setPoint( pt );

                }


            }

        }

    }

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

void AdjacentFrame::matchFrames()
{
    auto frame1 = std::dynamic_pointer_cast< FeatureFrame >( this->previousFrame() );
    auto frame2 = std::dynamic_pointer_cast< FeatureFrame >( this->nextFrame() );

    if ( frame1 && frame2 ) {

        std::vector< cv::DMatch > matches;

        m_matcher.match( frame1->m_keyPoints, frame1->m_descriptors, frame2->m_keyPoints, frame2->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto point1 = frame1->framePoint( i.queryIdx );
            auto point2 = frame2->framePoint( i.trainIdx );

            double lenght = cv::norm( point1->point() - point2->point() );

            if ( lenght >= m_minLenght ) {

                point1->setNextPoint( point2 );
                point2->setPrevPoint( point1 );

                auto worlPoint = point1->worldPoint();

                if ( worlPoint )
                    point2->setWorldPoint( worlPoint );

            }

        }

    }

}

bool AdjacentFrame::track()
{
    std::vector< cv::Point3f > worldPoints;
    std::vector< cv::Point2f > framePoints;

    auto prevFrame = previousFrame();
    auto frame = nextFrame();

    for ( auto &i : frame->framePoints() ) {
        if ( i && i->worldPoint() ) {
            worldPoints.push_back( i->worldPoint()->point() );
            framePoints.push_back( i->point() );

        }

    }

    if ( worldPoints.size() < m_minPnpPoints )
        return false;

    cv::Mat rvec;
    cv::Mat tvec;

    cv::solvePnPRansac( worldPoints, framePoints, prevFrame->cameraMatrix(), cv::noArray(), rvec, tvec );

    cv::Mat rmat;
    cv::Rodrigues( rvec, rmat );

    frame->setCameraMatrix( prevFrame->cameraMatrix() );
    frame->setTranslation( tvec );
    frame->setRotation( rmat );

    return true;

}

AdjacentFrame::MonoFramePtr AdjacentFrame::previousFrame() const
{
    return frame1();
}

AdjacentFrame::MonoFramePtr AdjacentFrame::nextFrame() const
{
    return frame2();
}

std::vector< AdjacentPoint > AdjacentFrame::adjacentPoints() const
{
    auto prevFrame = this->previousFrame();

    if (prevFrame)
        return prevFrame->adjacentPoints();
    else
        return std::vector< AdjacentPoint >();

}

CvImage AdjacentFrame::drawTrack( const CvImage &image ) const
{
    CvImage ret;

    image.copyTo( ret );

    if ( m_frame2 ) {

        for ( auto &i : m_frame2->framePoints() )
            if ( i )
                i->drawTrack( &ret );

        for ( auto &i : m_frame2->framePoints() )
            if ( i && i->prevPoint() )
                drawFeaturePoint( &ret, i->point(), 2 );

    }

    return ret;

}

// WorldAdjacentFrame
WorldAdjacentFrame::WorldAdjacentFrame( const WorldPtr &parentWorld )
    : m_parentWorld( parentWorld )
{
}

WorldAdjacentFrame::FramePtr WorldAdjacentFrame::create( const WorldPtr &parentWorld )
{
    return FramePtr( new WorldAdjacentFrame( parentWorld ) );
}

const WorldAdjacentFrame::WorldPtr WorldAdjacentFrame::parentWorld() const
{
    return m_parentWorld.lock();
}

void WorldAdjacentFrame::triangulatePoints()
{
    auto world = parentWorld();

    if ( world ) {

        cv::Mat points3d;

        auto adjacentPoints = this->adjacentPoints();

        cv::Mat_< float > prevPoints( 2, adjacentPoints.size() ),
                          nextPoints( 2, adjacentPoints.size() );

        for ( size_t i = 0; i < adjacentPoints.size(); ++i ) {
            prevPoints.row( 0 ).col( i ) = adjacentPoints[ i ].previousPoint().x;
            prevPoints.row( 1 ).col( i ) = adjacentPoints[ i ].previousPoint().y;
            nextPoints.row( 0 ).col( i ) = adjacentPoints[ i ].nextPoint().x;
            nextPoints.row( 1 ).col( i ) = adjacentPoints[ i ].nextPoint().y;

        }

        cv::triangulatePoints( previousFrame()->projectionMatrix(), nextFrame()->projectionMatrix(), prevPoints, nextPoints, points3d );

        for ( size_t i = 0; i < adjacentPoints.size(); ++i ) {

            auto w = points3d.at< float >( 3, i );

            auto x = points3d.at< float >( 0, i ) / w;
            auto y = points3d.at< float >( 1, i ) / w;
            auto z = points3d.at< float >( 2, i ) / w;

            auto pt = cv::Vec3f( x, y, z );

            if ( std::abs( w ) > FLOAT_EPS ) {
                if ( !adjacentPoints[i].previousFramePoint()->worldPoint() && !adjacentPoints[i].nextFramePoint()->worldPoint() ) {
                    auto worldPoint = world->createWorldPoint( pt, adjacentPoints[i].previousFramePoint()->color() );
                    adjacentPoints[i].previousFramePoint()->setWorldPoint( worldPoint );
                    adjacentPoints[i].nextFramePoint()->setWorldPoint( worldPoint );

                }
                else {
                    if ( !adjacentPoints[i].previousFramePoint()->worldPoint() )
                        adjacentPoints[i].previousFramePoint()->setWorldPoint( adjacentPoints[i].nextFramePoint()->worldPoint() );
                    else if ( !adjacentPoints[i].nextFramePoint()->worldPoint() )
                        adjacentPoints[i].nextFramePoint()->setWorldPoint( adjacentPoints[i].previousFramePoint()->worldPoint() );

                    adjacentPoints[i].previousFramePoint()->worldPoint()->setPoint( pt );

                }


            }

        }

    }

}

}

