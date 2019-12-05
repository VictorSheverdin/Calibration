#include "src/common/precompiled.h"

#include "frame.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

#include "map.h"

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
}

std::vector< cv::Point2f > MonoFrame::points() const
{
    std::vector< cv::Point2f > ret;

    auto framePoints = this->framePoints();

    for ( auto &i : framePoints )
        if ( i )
            ret.push_back( i->point() );

    return ret;

}

std::vector< StereoPoint > MonoFrame::stereoPoints() const
{
    std::vector< StereoPoint > ret;

    auto framePoints = this->framePoints();

    for ( auto &i : framePoints ) {
        if ( i && i->stereoPoint() )
            ret.push_back( StereoPoint( i, i->stereoPoint() ));
    }

    return ret;
}

std::vector< AdjacentPoint > MonoFrame::adjacentPoints() const
{
    std::vector< AdjacentPoint > ret;

    auto framePoints = this->framePoints();

    for ( auto &i : framePoints ) {
        if ( i && i->nextPoint() )
            ret.push_back( AdjacentPoint( i, i->nextPoint() ));
    }

    return ret;
}

std::vector< MonoFrame::PointPtr > MonoFrame::trackPoints() const
{
    std::vector< PointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        if ( i && i->mapPoint() ) {
            ret.push_back( i );

        }

    }

    return ret;

}

int MonoFrame::trackPointsCount() const
{
    return trackPoints().size();
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

cv::Point3d MonoFrame::point() const
{
    auto t = translation();

     return  cv::Point3d( t.at< double >( 0, 0 ), t.at< double >( 1, 0 ), t.at< double >( 2, 0 ) );
}

// ProcessedFrame
DaisyProcessor ProcessedFrame::m_descriptorProcessor;
GFTTProcessor ProcessedFrame::m_keypointProcessor;

const double ProcessedFrame::m_minCameraDistanceFactor = 5.0;
const double ProcessedFrame::m_minPointsDistance = 10.0;

ProcessedFrame::ProcessedFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
    initialize();
}

void ProcessedFrame::initialize()
{
}

std::vector< ProcessedFrame::PointPtr > ProcessedFrame::framePoints() const
{
    std::vector< PointPtr > ret;

    for ( auto &i : m_points )
        ret.push_back( i.second );

    return ret;

}

ProcessedFrame::MapPtr ProcessedFrame::parentMap() const
{
    return m_parentMap.lock();
}

ProcessedFrame::ProcessedPointPtr &ProcessedFrame::framePoint( const size_t index )
{
    if ( m_points.find( index ) == m_points.end() )
        createFramePoint( index, m_colors[ index ] );

    return m_points.at( index );
}

const ProcessedFrame::ProcessedPointPtr &ProcessedFrame::framePoint( const size_t index ) const
{
    return m_points.at( index );
}

ProcessedFrame::FramePtr ProcessedFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new ProcessedFrame( parentMap ) );
}

void ProcessedFrame::load( const CvImage &image )
{
    m_image = image;
}

void ProcessedFrame::extractKeyPoints()
{
    m_points.clear();

    m_keypointProcessor.extractKeypoints( m_image, &m_keyPoints );

    m_colors.clear();

    for ( size_t i = 0; i < m_keyPoints.size(); ++i ) {
        m_colors.push_back( m_image.at< cv::Vec3b >( m_keyPoints[ i ].pt ) );

    }

}

void ProcessedFrame::extractDescriptors()
{
    m_descriptorProcessor.extractDescriptors( m_image, m_keyPoints, &m_descriptors );
}

const CvImage ProcessedFrame::image() const
{
    return m_image;
}

void ProcessedFrame::clearImage()
{
    m_image = CvImage();
}

void ProcessedFrame::triangulatePoints()
{
    std::map< MonoFramePtr, std::vector< PointPtr > > frames;
    std::map< PointPtr, ProcessedPointPtr > points;

    for ( auto &i : m_points ) {

        auto ptr = i.second;

        if ( ptr ) {

            auto prevPtr = ptr->prevPoint();

            if ( prevPtr ) {

                PointPtr j;

                for ( j = prevPtr; j->prevPoint(); j = j->prevPoint() );

                if ( j ) {

                    auto parentFrame = j->parentFrame();

                    if ( parentFrame ) {
                        auto cameraDistance = cv::norm( point() - parentFrame->point() );
                        auto pointsDistance = cv::norm( j->point() - ptr->point() );

                        if ( cameraDistance > m_minCameraDistanceFactor * parentMap()->baselineLenght() && pointsDistance > m_minPointsDistance ) {
                            frames[ parentFrame ].push_back( j );
                            points[ j ] = ptr;

                        }

                    }

                }

            }

        }

    }

    for ( auto &i : frames ) {

        cv::Mat points3d;

        cv::Mat_< float > prevPoints( 2, i.second.size() ),
                          nextPoints( 2, i.second.size() );

        for ( size_t j = 0; j < i.second.size(); ++j ) {
            auto prevPoint = i.second[ j ]->point();
            prevPoints.row( 0 ).col( j ) = prevPoint.x;
            prevPoints.row( 1 ).col( j ) = prevPoint.y;

            auto nextPoint = points[ i.second[ j ] ]->point();
            nextPoints.row( 0 ).col( j ) = nextPoint.x;
            nextPoints.row( 1 ).col( j ) = nextPoint.y;

        }

        if ( !prevPoints.empty() && prevPoints.size() == nextPoints.size() ) {

            cv::triangulatePoints( i.first->projectionMatrix(), projectionMatrix(), prevPoints, nextPoints, points3d );

            for ( size_t j = 0; j < i.second.size(); ++j ) {

                auto w = points3d.at< float >( 3, j );

                auto x = points3d.at< float >( 0, j ) / w;
                auto y = points3d.at< float >( 1, j ) / w;
                auto z = points3d.at< float >( 2, j ) / w;

                auto pt = cv::Vec3f( x, y, z );

                if ( std::abs( w ) > FLOAT_EPS ) {

                    MapPointPtr mapPoint;

                    for ( auto k = i.second[ j ]; k; k = k->prevPoint() ) {


                        if ( k ) {
                            auto curMapPoint = k->mapPoint();

                            if ( curMapPoint )
                                mapPoint = curMapPoint;

                        }

                    }

                    if ( mapPoint ) {
                        mapPoint->setPoint( pt );
                        mapPoint->setColor( i.second[ j ]->color() );
                    }
                    else
                        i.second[ j ]->setMapPoint( parentMap()->createMapPoint( pt, i.second[ j ]->color() ) );

                }

            }

        }

    }

}

const std::vector< cv::KeyPoint > &ProcessedFrame::keyPoints() const
{
    return m_keyPoints;
}

CvImage ProcessedFrame::drawKeyPoints() const
{
    if ( !m_image.empty() ) {
        CvImage ret;
        m_image.copyTo( ret );
        drawFeaturePoints( &ret, m_keyPoints );

        drawLabel( &ret, "Keypoints cout: " + std::to_string( m_keyPoints.size() ), ret.height() / 70 );

        return ret;

    }

    return CvImage();

}

CvImage ProcessedFrame::drawTracks() const
{
    CvImage ret;

    m_image.copyTo( ret );

    auto points = this->framePoints();

#pragma omp parallel for
    for ( size_t i = 0; i < points.size(); ++i )
        if ( points[ i ] && ( points[ i ]->prevPoint() || points[i]->nextPoint() ) )
            if ( points[i]->mapPoint() )
                points[ i ]->drawTrack( &ret );
            else
                points[ i ]->drawTrack( &ret, cv::Scalar( 255, 0, 0, 255 ) );


#pragma omp parallel for
    for ( size_t i = 0; i < points.size(); ++i )
        if ( points[ i ] && ( points[ i ]->prevPoint() || points[i]->nextPoint() ) )
            drawFeaturePoint( &ret, points[ i ]->point(), 2 );

    drawLabel( &ret, "Tracks cout: " + std::to_string( points.size() ), ret.height() / 70 );

    return ret;

}

ProcessedFrame::ProcessedPointPtr ProcessedFrame::createFramePoint( const size_t keyPointIndex , const cv::Scalar &color )
{
    if ( keyPointIndex >= m_keyPoints.size() )
        return ProcessedPointPtr();

    auto point = ProcessedPoint::create( shared_from_this(), keyPointIndex );

    m_points[ keyPointIndex ] = point;

    return point;

}

void ProcessedFrame::setMaxFeatures( const int value )
{
    m_keypointProcessor.setMaxFeatures( value );
}

int ProcessedFrame::maxFeatures()
{
    return m_keypointProcessor.maxFeatures();
}

// Frame
Frame::Frame()
{
}

std::vector< Frame::PointPtr > Frame::framePoints() const
{
    std::vector< PointPtr > ret;

    for ( auto &i : m_points )
        ret.push_back( i );

    return ret;
}

Frame::FramePtr Frame::create()
{
    return FramePtr( new Frame() );
}

void Frame::replace( const ProcessedFramePtr &frame )
{
    if ( frame ) {

        setProjectionMatrix( frame->projectionMatrix() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            auto processedPoint = std::dynamic_pointer_cast< ProcessedPoint >( i );

            if ( processedPoint ) {
                auto point = createFramePoint( processedPoint->point(), processedPoint->color() );
                point->replace( processedPoint );

            }

        }

    }

}

Frame::FramePointPtr Frame::createFramePoint( const cv::Point2f &point, const cv::Scalar &color )
{
    auto ret = FramePoint::create( shared_from_this(), point, color );

    m_points.push_back( ret );

    return ret;

}

// DoubleFrame
DoubleFrame::DoubleFrame()
{
}

void DoubleFrame::setFrames( const MonoFramePtr &frame1, const MonoFramePtr &frame2 )
{
    m_frame1 = frame1;
    m_frame2 = frame2;
}

DoubleFrame::MonoFramePtr DoubleFrame::frame1() const
{
    return m_frame1;
}

DoubleFrame::MonoFramePtr DoubleFrame::frame2() const
{
    return m_frame2;
}

void DoubleFrame::setProjectionMatrix( const ProjectionMatrix &matrix1, const ProjectionMatrix &matrix2 )
{
    m_frame1->setProjectionMatrix( matrix1 );
    m_frame2->setProjectionMatrix( matrix2 );
}

// ProcessedDoubleFrame
FeatureMatcher ProcessedDoubleFrame::m_matcher;
OpticalMatcher ProcessedDoubleFrame::m_opticalMatcher;

ProcessedDoubleFrame::ProcessedDoubleFrame()
{
}

void ProcessedDoubleFrame::setFrames( const ProcessedFramePtr &frame1, const ProcessedFramePtr &frame2 )
{
    DoubleFrame::setFrames( frame1, frame2 );
}

ProcessedDoubleFrame::ProcessedFramePtr ProcessedDoubleFrame::frame1() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( DoubleFrame::frame1() );
}

ProcessedDoubleFrame::ProcessedFramePtr ProcessedDoubleFrame::frame2() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( DoubleFrame::frame2() );
}

void ProcessedDoubleFrame::extractKeyPoints()
{
    auto frame1 = this->frame1();
    auto frame2 = this->frame2();

    if ( frame1 )
        frame1->extractKeyPoints();

    if ( frame2 )
        frame2->extractKeyPoints();

}

void ProcessedDoubleFrame::extractDescriptors()
{
    auto frame1 = this->frame1();
    auto frame2 = this->frame2();

    if ( frame1 )
        frame1->extractDescriptors();

    if ( frame2 )
        frame2->extractDescriptors();

}

// ProcessedStereoFrame
const float ProcessedStereoFrame::m_maxYParallax = 2.0;

ProcessedStereoFrame::ProcessedStereoFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
}

ProcessedStereoFrame::FramePtr ProcessedStereoFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new ProcessedStereoFrame( parentMap ) );
}

void ProcessedStereoFrame::load( const CvImage &image1, const CvImage &image2 )
{
    auto leftFrame = ProcessedFrame::create( parentMap() );
    leftFrame->load( image1 );

    auto rightFrame = ProcessedFrame::create( parentMap() );
    rightFrame->load( image2 );

    setFrames( leftFrame, rightFrame );

}

ProcessedStereoFrame::ProcessedFramePtr ProcessedStereoFrame::leftFrame() const
{
    return frame1();
}

ProcessedStereoFrame::ProcessedFramePtr ProcessedStereoFrame::rightFrame() const
{
    return frame2();
}

void ProcessedStereoFrame::setProjectionMatrix( const ProjectionMatrix &leftMatrix, const ProjectionMatrix &rightMatrix )
{
    ProcessedDoubleFrame::setProjectionMatrix( leftMatrix, rightMatrix );
}

cv::Mat ProcessedStereoFrame::matchOptical()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        std::vector< cv::DMatch > matches;

        auto fmat = m_opticalMatcher.match( leftFrame->image(), leftFrame->m_keyPoints, rightFrame->image(), rightFrame->m_keyPoints, &matches );

        for ( auto &i : matches ) {
            auto leftFramePoint = leftFrame->framePoint( i.queryIdx );
            auto rightFramePoint = rightFrame->framePoint( i.trainIdx );

            if ( !leftFramePoint->stereoPoint() || !rightFramePoint->stereoPoint() ) {

                auto leftPoint = leftFramePoint->point();
                auto rightPoint = rightFramePoint->point();

                if ( leftPoint.x > rightPoint.x && fabs( leftPoint.y - rightPoint.y ) < m_maxYParallax ) {

                    double lenght = cv::norm( leftPoint - rightPoint );

                    if ( lenght > m_minLenght ) {

                        leftFramePoint->setStereoPoint( rightFramePoint );
                        rightFramePoint->setStereoPoint( leftFramePoint );

                    }

                }

            }

        }

        return fmat;

    }

    return cv::Mat();

}

cv::Mat ProcessedStereoFrame::matchFeatures()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        std::vector< cv::DMatch > matches;

        auto fmat = m_matcher.match( leftFrame->m_keyPoints, leftFrame->m_descriptors, rightFrame->m_keyPoints, rightFrame->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto leftPoint = leftFrame->framePoint( i.queryIdx );
            auto rightPoint = rightFrame->framePoint( i.trainIdx );

            double lenght = cv::norm( leftPoint->point() - rightPoint->point() );

            if ( lenght >= m_minLenght ) {
                leftPoint->setStereoPoint( rightPoint );
                rightPoint->setStereoPoint( leftPoint );

            }

        }

        return fmat;

    }

    return cv::Mat();

}

void ProcessedStereoFrame::clearImages()
{
    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame )
        leftFeatureFrame->clearImage();

    if ( rightFeatureFrame )
        rightFeatureFrame->clearImage();

}

std::vector< StereoPoint > ProcessedStereoFrame::stereoPoints() const
{
    auto leftFrame = this->leftFrame();

    if (leftFrame)
        return leftFrame->stereoPoints();
    else
        return std::vector< StereoPoint >();

}

int ProcessedStereoFrame::stereoPointsCount() const
{
    return stereoPoints().size();
}

CvImage ProcessedStereoFrame::drawKeyPoints() const
{
    CvImage leftImage;
    CvImage rightImage;

    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame )
        leftImage = leftFeatureFrame->drawKeyPoints();

    if ( rightFeatureFrame )
        rightImage = rightFeatureFrame->drawKeyPoints();

    return stackImages( leftImage, rightImage );
}

CvImage ProcessedStereoFrame::drawStereoPoints() const
{
    CvImage ret;

    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame && rightFeatureFrame ) {

        auto leftImage = leftFeatureFrame->image();
        auto rightImage = rightFeatureFrame->image();

        if ( !leftImage.empty() && !rightImage.empty() ) {

            ret = stackImages( leftImage, rightImage );

            auto stereoPoints = this->stereoPoints();

#pragma omp parallel for
            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {
                    auto rightPoint = stereoPoints[ i ].rightPoint();
                    rightPoint.x += leftImage.width();

                    drawLine( &ret, stereoPoints[ i ].leftPoint(), rightPoint );


            }

#pragma omp parallel for
            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {
                auto rightPoint = stereoPoints[ i ].rightPoint();
                rightPoint.x += leftImage.width();

                drawFeaturePoint( &ret, stereoPoints[ i ].leftPoint() );
                drawFeaturePoint( &ret, rightPoint );

            }

            drawLabel( &ret, "Stereo correspondencies cout: " + std::to_string( stereoPoints.size() ), ret.height() / 70 );


        }

    }

    return ret;

}

CvImage ProcessedStereoFrame::drawTracks() const
{
    CvImage leftImage;
    CvImage rightImage;

    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame )
        leftImage = leftFeatureFrame->drawTracks();

    if ( rightFeatureFrame )
        rightImage = rightFeatureFrame->drawTracks();

    return stackImages( leftImage, rightImage );

}

ProcessedStereoFrame::MapPtr ProcessedStereoFrame::parentMap() const
{
    return m_parentMap.lock();
}

bool ProcessedStereoFrame::triangulatePoints()
{
    auto map = parentMap();

    if ( map ) {

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

        if ( !leftPoints.empty() && leftPoints.size() == rightPoints.size() ) {

            cv::triangulatePoints( leftFrame()->projectionMatrix(), rightFrame()->projectionMatrix(), leftPoints, rightPoints, homogeneousPoints3d );

            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {

                auto w = homogeneousPoints3d.at< float >( 3, i );

                auto x = homogeneousPoints3d.at< float >( 0, i ) / w;
                auto y = homogeneousPoints3d.at< float >( 1, i ) / w;
                auto z = homogeneousPoints3d.at< float >( 2, i ) / w;

                auto pt = cv::Vec3f( x, y, z );

                if ( std::abs( w ) > FLOAT_EPS ) {

                    if ( !stereoPoints[i].leftFramePoint()->mapPoint() && !stereoPoints[i].rightFramePoint()->mapPoint() ) {

                        auto mapPoint = map->createMapPoint( pt, stereoPoints[i].leftFramePoint()->color() );

                        stereoPoints[i].leftFramePoint()->setMapPoint( mapPoint );
                        stereoPoints[i].rightFramePoint()->setMapPoint( mapPoint );


                    }
                    else {

                        if ( !stereoPoints[i].leftFramePoint()->mapPoint() )
                            stereoPoints[i].leftFramePoint()->setMapPoint( stereoPoints[i].rightFramePoint()->mapPoint() );

                        else if ( !stereoPoints[i].rightFramePoint()->mapPoint() )
                            stereoPoints[i].rightFramePoint()->setMapPoint( stereoPoints[i].leftFramePoint()->mapPoint() );

                        stereoPoints[i].leftFramePoint()->mapPoint()->setPoint( pt );

                    }

                }

            }

            return true;

        }

    }

    return false;

}


void ProcessedStereoFrame::clearMapPoints( const std::vector< MonoPointPtr > &points )
{
    auto parentMap = this->parentMap();

    if ( parentMap ) {

        for ( auto &i : points ) {

            if ( i ) {

                auto mapPoint = i->mapPoint();

                if ( mapPoint ) {

                    if ( !i->nextPoint() && !i->prevPoint() && ( !i->stereoPoint() || ( i->stereoPoint() &&
                         !i->stereoPoint()->nextPoint() && !i->stereoPoint()->prevPoint() ) ) ) {

                        parentMap->removeMapPoint( mapPoint );

                    }

                }

            }

        }

    }

}

void ProcessedStereoFrame::clearMapPoints()
{
    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame ) {
        clearMapPoints( leftFeatureFrame->framePoints() );

    }

    if ( rightFeatureFrame ) {
        clearMapPoints( rightFeatureFrame->framePoints() );

    }

}

void ProcessedStereoFrame::setMaxFeatures( const int value )
{
    ProcessedFrame::setMaxFeatures( value );
}

int ProcessedStereoFrame::maxFeatures()
{
    return ProcessedFrame::maxFeatures();
}


// AdjacentFrame
AdjacentFrame::AdjacentFrame()
{
}

AdjacentFrame::FramePtr AdjacentFrame::create()
{
    return FramePtr( new AdjacentFrame() );
}

cv::Mat AdjacentFrame::matchOptical()
{
    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        std::vector< cv::DMatch > matches;

        auto fmat = m_opticalMatcher.match( prevFrame->image(), prevFrame->m_keyPoints, nextFrame->image(), nextFrame->m_keyPoints, &matches );

        for ( auto &i : matches ) {
            auto point1 = prevFrame->framePoint( i.queryIdx );
            auto point2 = nextFrame->framePoint( i.trainIdx );

            double lenght = cv::norm( point1->point() - point2->point() );

            if ( lenght >= m_minLenght ) {

                point1->setNextPoint( point2 );
                point2->setPrevPoint( point1 );

                auto worlPoint = point1->mapPoint();

                if ( worlPoint )
                    point2->setMapPoint( worlPoint );

            }

        }

        return fmat;

    }

    return cv::Mat();

}

cv::Mat AdjacentFrame::matchFeatures()
{
    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        std::vector< cv::DMatch > matches;

        auto fmat = m_matcher.match( prevFrame->m_keyPoints, prevFrame->m_descriptors, nextFrame->m_keyPoints, nextFrame->m_descriptors, &matches );

        for ( auto &i : matches ) {
            auto point1 = prevFrame->framePoint( i.queryIdx );
            auto point2 = nextFrame->framePoint( i.trainIdx );

            double lenght = cv::norm( point1->point() - point2->point() );

            if ( lenght >= m_minLenght ) {

                point1->setNextPoint( point2 );
                point2->setPrevPoint( point1 );

                auto worlPoint = point1->mapPoint();

                if ( worlPoint )
                    point2->setMapPoint( worlPoint );

            }

        }

        return fmat;

    }

    return cv::Mat();

}

bool AdjacentFrame::track()
{
    std::vector< cv::Point3f > points3d;
    std::vector< cv::Point2f > points2d;

    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        auto points = nextFrame->trackPoints();

        for ( auto &i : points ) {
            points3d.push_back( i->mapPoint()->point() );
            points2d.push_back( i->point() );
        }

        if ( points3d.size() < m_minPnpPoints )
            return false;

        cv::Mat rvec;
        cv::Mat tvec;

        std::vector< int > inliers;

        cv::solvePnPRansac( points3d, points2d, prevFrame->cameraMatrix(), cv::noArray(), rvec, tvec, false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );

        std::set< int > inliersSet;

        for ( auto i : inliers )
            inliersSet.insert( i );

        for ( size_t i = 0; i < points.size(); ++i )

            if ( inliersSet.find( i ) == inliersSet.end() ) {

                auto prevPoint = points[i]->prevPoint();

                if ( prevPoint )
                    prevPoint->clearNextPoint();

                points[i]->clearPrevPoint();

            }

        cv::Mat rmat;
        cv::Rodrigues( rvec, rmat );

        nextFrame->setCameraMatrix( prevFrame->cameraMatrix() );
        nextFrame->setTranslation( tvec );
        nextFrame->setRotation( rmat );

    }

    return true;

}

AdjacentFrame::ProcessedFramePtr AdjacentFrame::previousFrame() const
{
    return frame1();
}

AdjacentFrame::ProcessedFramePtr AdjacentFrame::nextFrame() const
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

int AdjacentFrame::adjacentPointsCount() const
{
    return adjacentPoints().size();
}

// StereoFrame
StereoFrame::StereoFrame()
{
    initialize();
}

void StereoFrame::initialize()
{
    setFrames( Frame::create(), Frame::create() );
}

StereoFrame::StereoFramePtr StereoFrame::create()
{
    return StereoFramePtr( new StereoFrame() );
}


StereoFrame::FramePtr StereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< Frame >( DoubleFrame::frame1() );
}

StereoFrame::FramePtr StereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< Frame >( DoubleFrame::frame2() );
}

void StereoFrame::setProjectionMatrix( const ProjectionMatrix &leftMatrix, const ProjectionMatrix &rightMatrix )
{
    DoubleFrame::setProjectionMatrix( leftMatrix, rightMatrix );
}

void StereoFrame::replace( const ProcessedStereoFramePtr &frame )
{
    if ( frame ) {
        auto leftFrame = this->leftFrame();
        auto leftProcessedFrame = frame->leftFrame();
        auto rightFrame = this->rightFrame();
        auto rightProcessedFrame = frame->rightFrame();

        if ( leftFrame && leftProcessedFrame )
            leftFrame->replace( leftProcessedFrame );

        if ( rightFrame && rightProcessedFrame )
            rightFrame->replace( rightProcessedFrame );

    }
}

}

