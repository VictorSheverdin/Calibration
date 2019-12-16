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

const double ProcessedFrame::m_cameraDistanceMultiplier = 2.0;
const double ProcessedFrame::m_minPointsDistance = 2.0;

ProcessedFrame::ProcessedFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
    initialize();
}

void ProcessedFrame::initialize()
{
}

std::vector< MonoFrame::PointPtr > ProcessedFrame::framePoints() const
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

void ProcessedFrame::cleanPoints()
{
    for ( auto i = m_points.begin(); i != m_points.end(); ) {

        if ( !i->second->mapPoint() && !i->second->nextPoint() )
            i = m_points.erase( i );
        else
            ++i;

    }
}

void ProcessedFrame::cleanMapPoints()
{
    auto parentMap = this->parentMap();

    if ( parentMap ) {

        for ( auto &i : m_points ) {

            auto ptr = i.second;

            if ( ptr ) {

                auto mapPoint = ptr->mapPoint();

                if ( mapPoint ) {

                    if ( !ptr->nextPoint() && ptr->connectedPointsCount() < m_minConnectedPoints ) {

                        parentMap->removeMapPoint( mapPoint );

                    }

                }

            }

        }

    }
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

                        if ( cameraDistance > m_cameraDistanceMultiplier * parentMap()->baselineLenght() && pointsDistance > m_minPointsDistance ) {
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
            auto sourcePoint = i.second[ j ];

            auto prevPoint = sourcePoint->point();
            prevPoints.row( 0 ).col( j ) = prevPoint.x;
            prevPoints.row( 1 ).col( j ) = prevPoint.y;

            auto nextPoint = points[ sourcePoint ]->point();
            nextPoints.row( 0 ).col( j ) = nextPoint.x;
            nextPoints.row( 1 ).col( j ) = nextPoint.y;

        }

        if ( !prevPoints.empty() && prevPoints.size() == nextPoints.size() ) {

            auto prevProjectionMatrix = i.first->projectionMatrix();
            auto nextProjectionMatrix = projectionMatrix();

            cv::triangulatePoints( prevProjectionMatrix, nextProjectionMatrix, prevPoints, nextPoints, points3d );

            for ( size_t j = 0; j < i.second.size(); ++j ) {

                auto sourcePoint = i.second[ j ];

                auto processedPoint = points[ sourcePoint ];

                if ( processedPoint ) {

                    auto w = points3d.at< float >( 3, j );

                    auto x = points3d.at< float >( 0, j ) / w;
                    auto y = points3d.at< float >( 1, j ) / w;
                    auto z = points3d.at< float >( 2, j ) / w;

                    auto pt = cv::Point3d( x, y, z );

                    if ( std::abs( w ) > FLOAT_EPS ) {

                        cv::Mat pt4d( 4, 1, CV_64F );
                        points3d.col( j ).convertTo( pt4d, CV_64F );

                        cv::Mat prevReprojMat = prevProjectionMatrix * pt4d;
                        cv::Mat nextReprojMat = nextProjectionMatrix * pt4d;

                        auto prevW = prevReprojMat.at< double >( 2, 0 );
                        auto nextW = nextReprojMat.at< double >( 2, 0 );

                        if ( std::abs( prevW ) > DOUBLE_EPS && std::abs( nextW ) > DOUBLE_EPS ) {

                            cv::Point2d prevReprojPt( prevReprojMat.at< double >( 0, 0 ) / prevW,
                                                      prevReprojMat.at< double >( 1, 0 ) / prevW );

                            cv::Point2d nextReprojPt( nextReprojMat.at< double >( 0, 0 ) / nextW,
                                                      nextReprojMat.at< double >( 1, 0 ) / nextW );

                            auto prevNorm = cv::norm( prevReprojPt - cv::Point2d( sourcePoint->point() ) );
                            auto nextNorm = cv::norm( nextReprojPt - cv::Point2d( processedPoint->point() ) );

                            if ( prevNorm < MAX_REPROJECTION_ERROR && nextNorm < MAX_REPROJECTION_ERROR ) {

                                MapPointPtr mapPoint = processedPoint->mapPoint();

                                if ( !mapPoint ) {
                                    mapPoint = parentMap()->createMapPoint( pt, sourcePoint->color() );
                                }
                                else {
                                    mapPoint->setPoint( pt );
                                    mapPoint->setColor( sourcePoint->color() );

                                }

                                sourcePoint->setMapPoint( mapPoint );
                                processedPoint->setMapPoint( mapPoint );

                            }

                        }


                    }

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

        int radius = std::min( m_image.width(), m_image.height() ) / 500.0;

        CvImage ret;
        m_image.copyTo( ret );
        drawFeaturePoints( &ret, m_keyPoints, radius );

        drawLabel( &ret, "Keypoints cout: " + std::to_string( m_keyPoints.size() ), ret.height() / 70 );

        return ret;

    }

    return CvImage();

}

CvImage ProcessedFrame::drawTracks() const
{
    CvImage ret;

    if ( !m_image.empty() ) {

        m_image.copyTo( ret );

        auto points = this->framePoints();

#pragma omp parallel for
        for ( size_t i = 0; i < points.size(); ++i )
            if ( points[ i ] && ( points[ i ]->prevPoint() || points[i]->nextPoint() ) ) {
                if ( points[ i ]->mapPoint() )
                    points[ i ]->drawTrack( &ret );
                else
                    points[ i ]->drawTrack( &ret, cv::Scalar( 255, 0, 0, 255 ) );

            }

        int radius = std::min( ret.width(), ret.height() ) / 500.0;

#pragma omp parallel for
        for ( size_t i = 0; i < points.size(); ++i )
            if ( points[ i ] && ( points[ i ]->prevPoint() || points[ i ]->nextPoint() ) )
                drawFeaturePoint( &ret, points[ i ]->point(), radius );

        drawLabel( &ret, "Tracks count: " + std::to_string( points.size() ), ret.height() / 70 );

    }

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

std::vector< ProcessedFrame::ProcessedPointPtr > ProcessedFrame::posePoints() const
{
    std::vector< ProcessedPointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        auto processedPoint = std::dynamic_pointer_cast< ProcessedPoint >( i );

        if ( processedPoint && processedPoint->prevPoint() && processedPoint->mapPoint() && processedPoint->lastTriangulated() )

            ret.push_back( processedPoint );

    }

    return ret;

}

int ProcessedFrame::posePointsCount() const
{
    return posePoints().size();
}

std::vector< ProcessedFrame::ProcessedPointPtr > ProcessedFrame::trackedPoints() const
{
    std::vector< ProcessedPointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        auto processedPoint = std::dynamic_pointer_cast< ProcessedPoint >( i );

        if ( processedPoint && processedPoint->nextPoint() && ( processedPoint->mapPoint() || processedPoint->stereoPoint() ) )
            ret.push_back( processedPoint );

    }

    return ret;

}

std::vector< ProcessedFrame::ProcessedPointPtr > ProcessedFrame::pointsToTrack() const
{
    std::vector< ProcessedPointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        auto processedPoint = std::dynamic_pointer_cast< ProcessedPoint >( i );

        if ( processedPoint && ( processedPoint->mapPoint() || processedPoint->stereoPoint() )  )
            ret.push_back( processedPoint );

    }

    return ret;

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

void Frame::replaceAndClean( const ProcessedFramePtr &frame )
{
    if ( frame ) {

        setProjectionMatrix( frame->projectionMatrix() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            auto processedPoint = std::dynamic_pointer_cast< ProcessedPoint >( i );

            if ( processedPoint && ( processedPoint->mapPoint() || processedPoint->nextPoint() ) ) {
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

cv::Point3f DoubleFrame::center() const
{
    if ( m_frame1 && m_frame2 )
        return 0.5 * ( m_frame1->point() + m_frame2->point() );

    return cv::Point3f();
}

// ProcessedDoubleFrameBase
FeatureMatcher ProcessedDoubleFrameBase::m_featuresMatcher;
OpticalMatcher ProcessedDoubleFrameBase::m_opticalMatcher;

ProcessedDoubleFrameBase::ProcessedDoubleFrameBase()
{
    initialize();
}

void ProcessedDoubleFrameBase::initialize()
{
    // m_usedKeypointsCount = 0;
}
/*
size_t ProcessedDoubleFrameBase::usedKeypointsCount() const
{
    return m_usedKeypointsCount;
}
*/

//StereoFrameBase
StereoFrameBase::StereoFrameBase()
{
}

void StereoFrameBase::setFrames( const MonoFramePtr &leftFrame, const MonoFramePtr &rightFrame )
{
    DoubleFrame::setFrames( leftFrame, rightFrame );
}

StereoFrameBase::MonoFramePtr StereoFrameBase::leftFrame() const
{
    return frame1();
}

StereoFrameBase::MonoFramePtr StereoFrameBase::rightFrame() const
{
    return frame2();
}

// ProcessedStereoFrame
const float ProcessedStereoFrame::m_maxYParallax = 2.0;
const double ProcessedStereoFrame::m_minXDistasnce = 2.0;

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

void ProcessedStereoFrame::setFrames( const ProcessedFramePtr &leftFrame, const ProcessedFramePtr &rightFrame )
{
    StereoFrameBase::setFrames( leftFrame, rightFrame );
}

ProcessedStereoFrame::ProcessedFramePtr ProcessedStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( frame1() );
}

ProcessedStereoFrame::ProcessedFramePtr ProcessedStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( frame2() );
}

void ProcessedStereoFrame::setProjectionMatrix( const ProjectionMatrix &leftMatrix, const ProjectionMatrix &rightMatrix )
{
    StereoFrameBase::setProjectionMatrix( leftMatrix, rightMatrix );
}

cv::Mat ProcessedStereoFrame::matchOptical( const size_t count )
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        auto procCount = std::min( count, leftFrame->m_keyPoints.size() );

        std::vector< cv::DMatch > matches;

        std::vector< cv::KeyPoint > leftKeypoints( leftFrame->m_keyPoints.begin(), leftFrame->m_keyPoints.begin() + procCount );

        auto fmat = m_opticalMatcher.match( leftFrame->image(), leftKeypoints, rightFrame->image(), rightFrame->m_keyPoints, &matches );

        for ( auto &i : matches ) {

            auto leftFramePoint = leftFrame->framePoint( i.queryIdx );
            auto rightFramePoint = rightFrame->framePoint( i.trainIdx );

            if ( !leftFramePoint->stereoPoint() || !rightFramePoint->stereoPoint() ) {

                auto leftPoint = leftFramePoint->point();
                auto rightPoint = rightFramePoint->point();

                if ( fabs( leftPoint.y - rightPoint.y ) < m_maxYParallax && leftPoint.x - rightPoint.x >  m_minXDistasnce ) {

                    leftFramePoint->setStereoPoint( rightFramePoint );
                    rightFramePoint->setStereoPoint( leftFramePoint );

                }

            }

        }

        return fmat;

    }

    return cv::Mat();

}

cv::Mat ProcessedStereoFrame::matchFeatures( const size_t count )
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        std::vector< cv::DMatch > matches;

        extractDescriptors();

        auto fmat = m_featuresMatcher.match( leftFrame->m_keyPoints, leftFrame->m_descriptors, rightFrame->m_keyPoints, rightFrame->m_descriptors, &matches );

        for ( auto &i : matches ) {

            auto leftFramePoint = leftFrame->framePoint( i.queryIdx );
            auto rightFramePoint = rightFrame->framePoint( i.trainIdx );

            if ( !leftFramePoint->stereoPoint() || !rightFramePoint->stereoPoint() ) {

                auto leftPoint = leftFramePoint->point();
                auto rightPoint = rightFramePoint->point();

                if ( leftPoint.x > rightPoint.x && fabs( leftPoint.y - rightPoint.y ) < m_maxYParallax ) {

                    leftFramePoint->setStereoPoint( rightFramePoint );
                    rightFramePoint->setStereoPoint( leftFramePoint );

                }

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

            int radius = std::min( ret.width(), ret.height() ) / 500.0;

#pragma omp parallel for
            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {
                auto rightPoint = stereoPoints[ i ].rightPoint();
                rightPoint.x += leftImage.width();

                drawFeaturePoint( &ret, stereoPoints[ i ].leftPoint(), radius );
                drawFeaturePoint( &ret, rightPoint, radius );

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

            auto leftProjectionMatrix = leftFrame()->projectionMatrix();
            auto rightProjectionMatrix = rightFrame()->projectionMatrix();

            cv::triangulatePoints( leftProjectionMatrix, rightProjectionMatrix, leftPoints, rightPoints, homogeneousPoints3d );

            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {

                auto w = homogeneousPoints3d.at< float >( 3, i );

                auto x = homogeneousPoints3d.at< float >( 0, i ) / w;
                auto y = homogeneousPoints3d.at< float >( 1, i ) / w;
                auto z = homogeneousPoints3d.at< float >( 2, i ) / w;

                auto pt = cv::Point3d( x, y, z );

                if ( std::abs( w ) > FLOAT_EPS ) {

                    cv::Mat pt4d( 4, 1, CV_64F );
                    homogeneousPoints3d.col( i ).convertTo( pt4d, CV_64F );

                    cv::Mat leftReprojMat = leftProjectionMatrix * pt4d;
                    cv::Mat rightReprojMat = rightProjectionMatrix * pt4d;

                    auto leftW = leftReprojMat.at< double >( 2, 0 );
                    auto rightW = rightReprojMat.at< double >( 2, 0 );

                    if ( std::abs( leftW ) > DOUBLE_EPS && std::abs( rightW ) > DOUBLE_EPS ) {

                        cv::Point2d leftReprojPt( leftReprojMat.at< double >( 0, 0 ) / leftW,
                                                  leftReprojMat.at< double >( 1, 0 ) / leftW );

                        cv::Point2d rightReprojPt( rightReprojMat.at< double >( 0, 0 ) / rightW,
                                                  rightReprojMat.at< double >( 1, 0 ) / rightW );

                        auto leftNorm = cv::norm( leftReprojPt - cv::Point2d( stereoPoints[i].leftPoint() ) );
                        auto rightNorm = cv::norm( rightReprojPt - cv::Point2d( stereoPoints[i].rightPoint() ) );

                        if ( leftNorm < MAX_REPROJECTION_ERROR && rightNorm < MAX_REPROJECTION_ERROR ) {

                            auto leftFramePoint = stereoPoints[i].leftFramePoint();
                            auto rightFramePoint = stereoPoints[i].rightFramePoint();

                            MapPointPtr mapPoint;

                            if ( !leftFramePoint->mapPoint() && !rightFramePoint->mapPoint() ) {

                                mapPoint = map->createMapPoint( pt, leftFramePoint->color() );

                                leftFramePoint->setMapPoint( mapPoint );
                                rightFramePoint->setMapPoint( mapPoint );

                            }
                            else {

                                if ( !stereoPoints[i].leftFramePoint()->mapPoint() )
                                    leftFramePoint->setMapPoint( rightFramePoint->mapPoint() );

                                else if ( !rightFramePoint->mapPoint() )
                                    rightFramePoint->setMapPoint( leftFramePoint->mapPoint() );

                                mapPoint = leftFramePoint->mapPoint();

                                mapPoint->setPoint( pt );
                                mapPoint->setColor( leftFramePoint->color() );

                            }

                            auto leftNextPoint = leftFramePoint->nextPoint();

                            if ( leftNextPoint )
                                leftNextPoint->setMapPoint( mapPoint );

                            auto rightNextPoint = rightFramePoint->nextPoint();

                            if ( rightNextPoint )
                                rightNextPoint->setMapPoint( mapPoint );

                        }

                    }

                }

            }

            return true;

        }

    }

    return false;

}

void ProcessedStereoFrame::cleanMapPoints()
{
    auto leftFeatureFrame = this->leftFrame();

    if ( leftFeatureFrame ) {
        leftFeatureFrame->cleanMapPoints();

    }

}

void ProcessedStereoFrame::extractKeyPoints()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame )
        leftFrame->extractKeyPoints();

    if ( rightFrame )
        rightFrame->extractKeyPoints();

}

void ProcessedStereoFrame::extractDescriptors()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame )
        leftFrame->extractDescriptors();

    if ( rightFrame )
        rightFrame->extractDescriptors();

}

size_t ProcessedStereoFrame::leftKeyPointsCount() const
{
    return leftFrame()->keyPoints().size();
}

// AdjacentFrame
AdjacentFrame::AdjacentFrame()
{
}

AdjacentFrame::FramePtr AdjacentFrame::create()
{
    return FramePtr( new AdjacentFrame() );
}

cv::Mat AdjacentFrame::trackOptical()
{
    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        auto trackPoints = prevFrame->pointsToTrack();

        std::vector< cv::KeyPoint > prevKeypoints;

        for ( auto &i : trackPoints )
            prevKeypoints.push_back( i->keyPoint() );

        std::vector< cv::DMatch > matches;

        auto fmat = m_opticalMatcher.match( prevFrame->image(), prevKeypoints, nextFrame->image(), nextFrame->m_keyPoints, &matches );

        for ( auto &i : matches ) {

            auto prevPoint = trackPoints[ i.queryIdx ];
            auto nextPoint = nextFrame->framePoint( i.trainIdx );

            prevPoint->setNextPoint( nextPoint );
            nextPoint->setPrevPoint( prevPoint );

            auto worlPoint = prevPoint->mapPoint();

            if ( worlPoint )
                nextPoint->setMapPoint( worlPoint );

        }

        return fmat;

    }

    return cv::Mat();
}

cv::Mat AdjacentFrame::trackFeatures( const size_t count )
{
    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        std::vector< cv::DMatch > matches;

        extractDescriptors();

        auto fmat = m_featuresMatcher.match( prevFrame->m_keyPoints, prevFrame->m_descriptors, nextFrame->m_keyPoints, nextFrame->m_descriptors, &matches );

        for ( auto &i : matches ) {

            auto point1 = prevFrame->framePoint( i.queryIdx );
            auto point2 = nextFrame->framePoint( i.trainIdx );

            point1->setNextPoint( point2 );
            point2->setPrevPoint( point1 );

            auto worlPoint = point1->mapPoint();

            if ( worlPoint )
                point2->setMapPoint( worlPoint );


        }

        return fmat;

    }

    return cv::Mat();

}

double AdjacentFrame::recoverPose()
{
    std::vector< cv::Point3f > points3d;
    std::vector< cv::Point2f > points2d;

    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        auto points = nextFrame->posePoints();

        for ( auto &i : points ) {
            points3d.push_back( i->mapPoint()->point() );
            points2d.push_back( i->point() );
        }

        if ( points3d.size() < MIN_PNP_POINTS_COUNT )
            return false;

        cv::Mat rvec;
        cv::Mat tvec;

        std::vector< int > inliers;

        cv::solvePnPRansac( points3d, points2d, prevFrame->cameraMatrix(), cv::noArray(), rvec, tvec, false,
                                        200, MAX_REPROJECTION_ERROR, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );

        std::set< int > inliersSet;

        for ( auto i : inliers )
            inliersSet.insert( i );

        for ( size_t i = 0; i < points.size(); ++i )

            if ( inliersSet.find( i ) == inliersSet.end() ) {

                auto prevPoint = points[i]->prevPoint();

                if ( prevPoint )
                    prevPoint->clearNextPoint();

                points[i]->clearPrevPoint();

                points[i]->clearMapPoint();

            }

        cv::Mat rmat;
        cv::Rodrigues( rvec, rmat );

        nextFrame->setCameraMatrix( prevFrame->cameraMatrix() );
        nextFrame->setTranslation( tvec );
        nextFrame->setRotation( rmat );

        return static_cast< double >( inliers.size() ) / points.size();

    }

    return 0;

}

void AdjacentFrame::setFrames( const ProcessedFramePtr &prevFrame, const ProcessedFramePtr &nextFrame )
{
    DoubleFrame::setFrames( prevFrame, nextFrame );
}

AdjacentFrame::ProcessedFramePtr AdjacentFrame::previousFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( frame1() );
}

AdjacentFrame::ProcessedFramePtr AdjacentFrame::nextFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( frame2() );
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

std::vector< AdjacentFrame::ProcessedPointPtr > AdjacentFrame::posePoints() const
{
    return nextFrame()->posePoints();
}

int AdjacentFrame::posePointsCount() const
{
    return nextFrame()->posePointsCount();
}

std::vector< AdjacentFrame::ProcessedPointPtr > AdjacentFrame::trackedPoints() const
{
    return previousFrame()->trackedPoints();
}

int AdjacentFrame::trackedPointsCount() const
{
    return trackedPoints().size();
}

void AdjacentFrame::extractDescriptors()
{
    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame )
        prevFrame->extractDescriptors();

    if ( nextFrame )
        nextFrame->extractDescriptors();

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

void StereoFrame::setFrames( const FramePtr &leftFrame, const FramePtr &rightFrame )
{
    DoubleFrame::setFrames( leftFrame, rightFrame );
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

void StereoFrame::replaceAndClean( const ProcessedStereoFramePtr &frame )
{
    if ( frame ) {
        auto leftFrame = this->leftFrame();
        auto leftProcessedFrame = frame->leftFrame();
        auto rightFrame = this->rightFrame();
        auto rightProcessedFrame = frame->rightFrame();

        if ( leftFrame && leftProcessedFrame )
            leftFrame->replaceAndClean( leftProcessedFrame );

        if ( rightFrame && rightProcessedFrame )
            rightFrame->replaceAndClean( rightProcessedFrame );

    }

}

}

