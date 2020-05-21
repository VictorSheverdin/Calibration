#include "src/common/precompiled.h"

#include "frame.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

#include "map.h"
#include "world.h"

#include "optimizer.h"

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

cv::Point3d MonoFrame::point() const
{
    auto t = translation();

    return  cv::Point3d( t.at< double >( 0, 0 ), t.at< double >( 1, 0 ), t.at< double >( 2, 0 ) );
}

g2o::SE3Quat MonoFrame::se3Pose() const
{
    auto rMat = rotation();
    auto tMat = translation();

    Eigen::Matrix< double, 3, 3 > r;

    r << rMat.at< double >( 0, 0 ), rMat.at< double >( 0, 1 ), rMat.at< double >( 0, 2 ),
         rMat.at< double >( 1, 0 ), rMat.at< double >( 1, 1 ), rMat.at< double >( 1, 2 ),
         rMat.at< double >( 2, 0 ), rMat.at< double >( 2, 1 ), rMat.at< double >( 2, 2 );

    Eigen::Matrix< double, 3, 1 > t( tMat.at< double >( 0, 0 ), tMat.at< double >( 1, 0 ), tMat.at< double >( 2, 0 ) );

    return g2o::SE3Quat( r, t );

}

void MonoFrame::setSe3Pose( const g2o::SE3Quat &pose )
{
    auto homogeniousMatrix = pose.to_homogeneous_matrix();

    auto rMat = rotation();
    auto tMat = translation();

    for ( auto i = 0; i < 3; ++i )
        for ( auto j = 0; j < 3; ++j )
            rMat.at< double >( i, j ) = homogeniousMatrix( i, j );

    for ( auto i = 0; i < 3; ++i )
            tMat.at< double >( i, 0 ) = homogeniousMatrix( i, 3 );

    setRotation( rMat );
    setTranslation( tMat );

}

// ProcessedFrame
const double ProcessedFrame::m_minPointsDistance = 10.0;

ProcessedFrame::ProcessedFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
}

size_t ProcessedFrame::extractedPointsCount() const
{
    return extractedPoints().size();
}

ProcessedFrame::MapPtr ProcessedFrame::parentMap() const
{
    return m_parentMap.lock();
}

ProcessedFrame::WorldPtr ProcessedFrame::parentWorld() const
{
    return parentMap()->parentWorld();
}

void ProcessedFrame::load( const CvImage &image )
{
    m_image = image;
}

const CvImage &ProcessedFrame::image() const
{
    return m_image;
}

void ProcessedFrame::clearImage()
{
    m_image.release();
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

CvImage ProcessedFrame::drawExtractedPoints() const
{
    if ( !m_image.empty() ) {

        int radius = std::max( std::min( m_image.width(), m_image.height() ) / 300.0, 1.0 );

        CvImage ret;
        m_image.copyTo( ret );

        auto points = extractedPoints();

        drawFeaturePoints( &ret, points, radius );

        drawLabel( &ret, "Extracted points count: " + std::to_string( points.size() ), ret.height() / 70 );

        return ret;

    }

    return CvImage();

}

int ProcessedFrame::triangulatePoints()
{
    int ret = 0;

    std::map< FramePtr, std::vector< PointPtr > > frames;
    std::map< PointPtr, PointPtr > points;

    auto framePoints = this->framePoints();

    auto maxReprojectionError = parentWorld()->maxReprojectionError();

    for ( auto &i : framePoints ) {

        if ( i ) {

            auto prevPtr = i->prevPoint();

            if ( prevPtr ) {

                PointPtr j;

                for ( j = prevPtr; j->prevPoint(); j = j->prevPoint() );

                if ( j ) {

                    auto parentFrame = j->parentFrame();

                    if ( parentFrame ) {

                        auto cameraDistance = cv::norm( point() - parentFrame->point() );
                        auto pointsDistance = cv::norm( j->point() - i->point() );

                        if ( cameraDistance > parentMap()->minTriangulateCameraDistance() && pointsDistance > m_minPointsDistance ) {
                            frames[ parentFrame ].push_back( j );
                            points[ j ] = i;

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

            // auto prevCameraPlane = i.first->zeroPlane();
            // auto nextCameraPlane = zeroPlane();

            cv::triangulatePoints( prevProjectionMatrix, nextProjectionMatrix, prevPoints, nextPoints, points3d );

            for ( size_t j = 0; j < i.second.size(); ++j ) {

                auto sourcePoint = i.second[ j ];

                auto processedPoint = points[ sourcePoint ];

                if ( processedPoint ) {

                    auto w = points3d.at< float >( 3, j );

                    if ( std::abs( w ) > FLOAT_EPS ) {

                        auto x = points3d.at< float >( 0, j ) / w;
                        auto y = points3d.at< float >( 1, j ) / w;
                        auto z = points3d.at< float >( 2, j ) / w;

                        // TODO: понять почему проверка не проходит
                        // if ( prevCameraPlane.value( x - i.first->x(), y - i.first->y(), z - i.first->z() ) > 0 ||
                        //     nextCameraPlane.value( x - this->x(), y - this->y(), z - this->z() ) > 0 ) {

                            auto pt = cv::Point3d( x, y, z );

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

                                if ( prevNorm < maxReprojectionError && nextNorm < maxReprojectionError ) {

                                    ++ret;

                                    auto mapPoint = processedPoint->mapPoint();

                                    if ( !mapPoint ) {
                                        mapPoint = parentMap()->createMapPoint( pt, sourcePoint->color() );
                                    }
                                    else {
                                        mapPoint->setPoint( pt );
                                        mapPoint->setColor( sourcePoint->color() );

                                    }

                                    for ( PointPtr k = processedPoint; k; k = k->prevPoint() ) {
                                        k->setMapPoint( mapPoint );
                                        auto stereoPoint = k->stereoPoint();
                                        if ( stereoPoint )
                                            stereoPoint->setMapPoint( mapPoint );

                                    }

                                }

                            }

                        // }

                    }

                }

            }

        }

    }

    return ret;

}

void ProcessedFrame::cleanMapPoints()
{
    auto parentMap = this->parentMap();

    auto points = framePoints();

    for ( auto &i : points ) {

        if ( i ) {

            auto mapPoint = i->mapPoint();

            if ( mapPoint ) {

                if ( !i->nextPoint() && i->connectedPointsCount() < m_minConnectedPoints ) {

                    parentMap->removeMapPoint( mapPoint );

                }

            }

        }

    }

}

std::vector< ProcessedFrame::PointPtr > ProcessedFrame::posePoints() const
{
    std::vector< PointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        if ( i && i->prevPoint() && i->mapPoint() )
            ret.push_back( i );

    }

    return ret;

}

int ProcessedFrame::posePointsCount() const
{
    return posePoints().size();
}

std::vector< ProcessedFrame::PointPtr > ProcessedFrame::trackFramePoints() const
{
    std::vector< PointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        if ( i && !i->mapPoint() && !i->stereoPoint() )
            ret.push_back( i );

    }

    return ret;

}

int ProcessedFrame::trackFramePointsCount() const
{
    return trackFramePoints().size();
}

std::vector< ProcessedFrame::PointPtr > ProcessedFrame::trackedPoints() const
{
    std::vector< PointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        if ( i && i->nextPoint() && ( i->mapPoint() || i->stereoPoint() ) )
            ret.push_back( i );

    }

    return ret;

}

int ProcessedFrame::trackedPointsCount() const
{
    return trackedPoints().size();
}

void ProcessedFrame::setImagePyramid( const std::vector<cv::Mat> &value )
{
    m_imagePyramid = value;
}

const std::vector< cv::Mat > &ProcessedFrame::imagePyramid() const
{
    return m_imagePyramid;
}

// FlowFrame
FlowFrame::FlowFrame( const MapPtr &parentMap )
    : ProcessedFrame( parentMap )
{
    initialize();
}

void FlowFrame::initialize()
{
    m_usePointsCount = 0;
}

FlowFrame::FramePtr FlowFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new FlowFrame( parentMap ) );
}

std::vector< FlowFrame::PointPtr > FlowFrame::framePoints() const
{
    return std::vector< PointPtr >( m_points.begin(), m_points.end() );
}

std::vector< cv::Point2f > FlowFrame::extractedPoints() const
{
    return m_extractedPoints;
}

std::list< FlowFrame::FlowPointPtr > &FlowFrame::flowPoints()
{
    return m_points;
}

const std::list< FlowFrame::FlowPointPtr > &FlowFrame::flowPoints() const
{
    return m_points;
}

void FlowFrame::extractPoints()
{
    parentWorld()->flowTracker().extractPoints( m_image, &m_extractedPoints );
}

void FlowFrame::createFramePoints( const size_t count )
{

    for ( size_t i = 0; i < count; ++i ) {

        if ( m_usePointsCount >= m_extractedPoints.size() )
            return;

        createFramePoint( m_usePointsCount );

        ++m_usePointsCount;

    }

}

FlowFrame::FlowPointPtr FlowFrame::createFramePoint( const size_t keyPointIndex )
{
    if ( keyPointIndex >= m_extractedPoints.size() )
        return FlowPointPtr();

    auto point = m_extractedPoints[ keyPointIndex ];

    auto flowPoint = FlowPoint::create( shared_from_this(), point, m_image.at< cv::Vec3b >( point ) );

    m_points.push_back( flowPoint );

    return flowPoint;

}

// FeatureFrame
FeatureFrame::FeatureFrame( const MapPtr &parentMap )
    : ProcessedFrame( parentMap )
{
    initialize();
}

void FeatureFrame::initialize()
{
}

std::vector< MonoFrame::PointPtr > FeatureFrame::framePoints() const
{
    std::vector< PointPtr > ret;

    for ( auto &i : m_points )
        ret.push_back( i.second );

    return ret;

}

std::vector< cv::Point2f > FeatureFrame::extractedPoints() const
{
    std::vector< cv::Point2f > ret;
    ret.reserve( m_keyPoints.size() );

    for ( auto &i : m_keyPoints )
        ret.push_back( i.pt );

    return ret;
}

std::vector< FeatureFrame::FeaturePointPtr > FeatureFrame::featurePoints() const
{
    std::vector< FeaturePointPtr > ret;

    auto framePoints = this->framePoints();

    ret.reserve( framePoints.size() );

    for ( auto &i : framePoints ) {

        auto processedPoint = std::dynamic_pointer_cast< FeaturePoint >( i );

        if ( processedPoint )
            ret.push_back( processedPoint );

    }

    return ret;

}

FeatureFrame::FeaturePointPtr &FeatureFrame::featurePoint( const size_t index )
{
    if ( !isFramePointExist( index ) )
        createFramePoint( index );

    return m_points.at( index );
}

const FeatureFrame::FeaturePointPtr &FeatureFrame::featurePoint( const size_t index ) const
{
    return m_points.at( index );
}

FeatureFrame::FramePtr FeatureFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new FeatureFrame( parentMap ) );
}

void FeatureFrame::load( const CvImage &image )
{
    ProcessedFrame::load( image );

    parentWorld()->featureTracker()->prepareFrame( this );

    m_points.clear();
    m_colors.clear();

    for ( size_t i = 0; i < m_keyPoints.size(); ++i )
        m_colors.push_back( m_image.at< cv::Vec3b >( m_keyPoints[ i ].pt ) );

}

FeatureFrame::FeaturePointPtr FeatureFrame::createFramePoint( const size_t keyPointIndex )
{
    if ( keyPointIndex >= m_keyPoints.size() )
        return FeaturePointPtr();

    auto point = FeaturePoint::create( shared_from_this(), keyPointIndex );

    m_points[ keyPointIndex ] = point;

    return point;

}

bool FeatureFrame::isFramePointExist( const size_t index ) const
{
    return m_points.find( index ) != m_points.end();
}

void FeatureFrame::createFramePoints( const size_t count )
{
    if ( count > 0 ) {

        size_t addCount = 0;

        for ( size_t i = 0; i < m_keyPoints.size(); ++i ) {
            if ( !isFramePointExist( i ) ) {
                createFramePoint( i );
                ++addCount;

                if ( addCount > count )
                    return;

            }

        }

    }

}

const std::vector< cv::KeyPoint > &FeatureFrame::keyPoints() const
{
    return m_keyPoints;
}

void FeatureFrame::setKeyPoints( const std::vector<cv::KeyPoint> &value )
{
    m_keyPoints = value;
}

const cv::Mat &FeatureFrame::descriptors() const
{
    return m_descriptors;
}

void FeatureFrame::setDescriptors( const cv::Mat &value )
{
    m_descriptors = value;
}

const cv::Mat &FeatureFrame::searchMatrix() const
{
    return m_searchMatrix;
}

void FeatureFrame::setSearchMatrix( const cv::Mat &value )
{
    m_searchMatrix = value;
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

void Frame::replace( const FeatureFramePtr &frame )
{
    if ( frame ) {

        setProjectionMatrix( frame->projectionMatrix() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            auto processedPoint = std::dynamic_pointer_cast< FeaturePoint >( i );

            if ( processedPoint ) {
                auto point = createFramePoint( processedPoint->point(), processedPoint->color() );
                point->replace( processedPoint );

            }

        }

    }

}

void Frame::replaceAndClean( const FeatureFramePtr &frame )
{
    if ( frame ) {

        setProjectionMatrix( frame->projectionMatrix() );

        auto points = frame->framePoints();

        for ( auto &i : points ) {

            auto processedPoint = std::dynamic_pointer_cast< FeaturePoint >( i );

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

//StereoFrameBase
StereoFrameBase::StereoFrameBase( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
}

void StereoFrameBase::setFrames( const MonoFramePtr &leftFrame, const MonoFramePtr &rightFrame )
{
    DoubleFrame::setFrames( leftFrame, rightFrame );
}

void StereoFrameBase::setProjectionMatrix( const StereoCameraMatrix &matrix )
{
    leftFrame()->setProjectionMatrix( matrix.leftProjectionMatrix() );
    rightFrame()->setProjectionMatrix( matrix.rightProjectionMatrix() );
}

StereoCameraMatrix StereoFrameBase::projectionMatrix() const
{
    return StereoCameraMatrix( leftFrame()->projectionMatrix(), rightFrame()->projectionMatrix() );
}

double StereoFrameBase::bf() const
{
    return leftFrame()->fx() * cv::norm( leftFrame()->translation() - rightFrame()->translation() );
}

StereoFrameBase::MonoFramePtr StereoFrameBase::leftFrame() const
{
    return frame1();
}

StereoFrameBase::MonoFramePtr StereoFrameBase::rightFrame() const
{
    return frame2();
}

void StereoFrameBase::setLeftSe3Pose( g2o::SE3Quat &pose )
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        auto rightPose = pose;
        rightPose.setTranslation( pose.translation() + rightFrame->se3Pose().translation() - leftFrame->se3Pose().translation() );

        leftFrame->setSe3Pose( pose );
        rightFrame->setSe3Pose( rightPose );

    }

}

StereoFrameBase::MapPtr StereoFrameBase::parentMap() const
{
    return m_parentMap.lock();
}

StereoFrameBase::WorldPtr StereoFrameBase::parentWorld() const
{
    return parentMap()->parentWorld();
}

// ProcessedStereoFrame
ProcessedStereoFrame::ProcessedStereoFrame( const MapPtr &parentMap )
    : StereoFrameBase( parentMap )
{
}

ProcessedStereoFrame::ProcessedFramePtr ProcessedStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( frame1() );
}

ProcessedStereoFrame::ProcessedFramePtr ProcessedStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< ProcessedFrame >( frame2() );
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

// FlowStereoFrame
FlowStereoFrame::FlowStereoFrame( const MapPtr &parentMap )
    : ProcessedStereoFrame( parentMap )
{
}

// FeatureStereoFrame
const double FeatureStereoFrame::m_minXDistasnce = 7.0;
const double FeatureStereoFrame::m_maxYDistasnce = 3.0;

FeatureStereoFrame::FeatureStereoFrame( const MapPtr &parentMap )
    : ProcessedStereoFrame( parentMap )
{
}

FeatureStereoFrame::FramePtr FeatureStereoFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new FeatureStereoFrame( parentMap ) );
}

void FeatureStereoFrame::load( const CvImage &image1, const CvImage &image2 )
{
    auto leftFrame = FeatureFrame::create( parentMap() );
    leftFrame->load( image1 );

    auto rightFrame = FeatureFrame::create( parentMap() );
    rightFrame->load( image2 );

    setFrames( leftFrame, rightFrame );

}

void FeatureStereoFrame::setFrames( const FeatureFramePtr &leftFrame, const FeatureFramePtr &rightFrame )
{
    StereoFrameBase::setFrames( leftFrame, rightFrame );
}

FeatureStereoFrame::FeatureFramePtr FeatureStereoFrame::leftFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( ProcessedStereoFrame::leftFrame() );
}

FeatureStereoFrame::FeatureFramePtr FeatureStereoFrame::rightFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( ProcessedStereoFrame::rightFrame() );
}

cv::Mat FeatureStereoFrame::match()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {

        auto trackPoints = leftFrame->featurePoints();

        std::vector< cv::DMatch > matches;

        auto fmat = parentWorld()->featureTracker()->match( leftFrame, rightFrame, &matches );

        for ( auto &i : matches ) {

            auto leftPoint = trackPoints[ i.queryIdx ];
            auto rightPoint = rightFrame->featurePoint( i.trainIdx );

            if ( leftPoint->point().x - rightPoint->point().x > m_minXDistasnce && std::abs( leftPoint->point().y - rightPoint->point().y ) < m_maxYDistasnce ) {

                leftPoint->setStereoPoint( rightPoint );
                rightPoint->setStereoPoint( leftPoint );

                auto worlPoint = leftPoint->mapPoint();

                if ( worlPoint )
                    rightPoint->setMapPoint( worlPoint );

            }

        }

        return fmat;

    }

    return cv::Mat();

}

std::vector< StereoPoint > FeatureStereoFrame::stereoPoints() const
{
    auto leftFrame = this->leftFrame();

    if (leftFrame)
        return leftFrame->stereoPoints();
    else
        return std::vector< StereoPoint >();

}

int FeatureStereoFrame::stereoPointsCount() const
{
    return stereoPoints().size();
}

CvImage FeatureStereoFrame::drawExtractedPoints() const
{
    CvImage leftImage;
    CvImage rightImage;

    auto leftFeatureFrame = this->leftFrame();
    auto rightFeatureFrame = this->rightFrame();

    if ( leftFeatureFrame )
        leftImage = leftFeatureFrame->drawExtractedPoints();

    if ( rightFeatureFrame )
        rightImage = rightFeatureFrame->drawExtractedPoints();

    return stackImages( leftImage, rightImage );
}

CvImage FeatureStereoFrame::drawStereoCorrespondences() const
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

            drawLabel( &ret, "Stereo correspondencies count: " + std::to_string( stereoPoints.size() ), ret.height() / 70 );


        }

    }

    return ret;

}

CvImage FeatureStereoFrame::drawTracks() const
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

int FeatureStereoFrame::triangulatePoints()
{
    int ret = 0;

    auto map = parentMap();

    if ( map ) {

        auto maxReprojectionError = parentWorld()->maxReprojectionError();

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

            auto leftCameraPlane = leftFrame()->plane();
            auto rightCameraPlane = rightFrame()->plane();

            cv::triangulatePoints( leftProjectionMatrix, rightProjectionMatrix, leftPoints, rightPoints, homogeneousPoints3d );

            for ( size_t i = 0; i < stereoPoints.size(); ++i ) {

                auto w = homogeneousPoints3d.at< float >( 3, i );

                if ( std::abs( w ) > FLOAT_EPS ) {

                    auto x = homogeneousPoints3d.at< float >( 0, i ) / w;
                    auto y = homogeneousPoints3d.at< float >( 1, i ) / w;
                    auto z = homogeneousPoints3d.at< float >( 2, i ) / w;

                    // TODO: Понять почему это не работает...
                    // if ( leftCameraPlane.value( x, y, z ) > 0 && rightCameraPlane.value( x, y, z ) > 0 ) {

                        auto pt = cv::Point3d( x, y, z );

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

                            if ( leftNorm < maxReprojectionError && rightNorm < maxReprojectionError ) {

                                ++ret;

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

                    // }

                }

            }

        }

    }

    return ret;

}

void FeatureStereoFrame::cleanMapPoints()
{
    auto leftFeatureFrame = this->leftFrame();

    if ( leftFeatureFrame ) {
        leftFeatureFrame->cleanMapPoints();

    }

}

// DenseFrameBase
DenseFrameBase::DenseFrameBase()
{
    initialize();
}

DenseFrameBase::~DenseFrameBase()
{
}

void DenseFrameBase::initialize()
{
}

void DenseFrameBase::setPoints( const std::list< ColorPoint3d > &list )
{
    m_points = list;
}

const std::list< ColorPoint3d > &DenseFrameBase::points() const
{
    return m_points;
}

void DenseFrameBase::createOptimizationGrid()
{
    m_optimizationGrid.clear();

    for ( auto j : m_points ) {
        auto point = j.point();
        m_optimizationGrid[ point.x * 10 ][ point.y * 10 ][ point.z * 10 ].push_back( j );

    }

}

void DenseFrameBase::setOptimizationGrid( const OptimizationGrid &grid )
{
    m_optimizationGrid = grid;
}

// ProcessedDenseFrame
ProcessedDenseFrame::ProcessedDenseFrame( const MapPtr &parentMap )
    : FeatureStereoFrame( parentMap )
{
}

ProcessedDenseFrame::FramePtr ProcessedDenseFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new ProcessedDenseFrame( parentMap ) );
}

void ProcessedDenseFrame::processDenseCloud()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {
        setPoints( parentMap()->parentWorld()->stereoProcessor().processPointList( leftFrame->image(), rightFrame->image() ) );

        // createOptimizationGrid();

    }

}

// AdjacentFrame
AdjacentFrame::AdjacentFrame( const MapPtr &parentMap )
    : m_parentMap( parentMap )
{
}

AdjacentFrame::FramePtr AdjacentFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new AdjacentFrame( parentMap ) );
}

cv::Mat AdjacentFrame::track()
{
    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        auto trackPoints = prevFrame->featurePoints();

        std::vector< cv::DMatch > matches;

        auto fmat = parentWorld()->featureTracker()->match( prevFrame, nextFrame, &matches );

        for ( auto &i : matches ) {

            auto prevPoint = trackPoints[ i.queryIdx ];
            auto nextPoint = nextFrame->featurePoint( i.trainIdx );

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

double AdjacentFrame::recoverPose()
{
    std::vector< cv::Point3f > points3d;
    std::vector< cv::Point2f > points2d;

    auto prevFrame = this->previousFrame();
    auto nextFrame = this->nextFrame();

    if ( prevFrame && nextFrame ) {

        auto parentWorld = this->parentWorld();

        auto maxReprojectionError = parentWorld->maxReprojectionError();

        auto points = nextFrame->posePoints();

        for ( auto &i : points ) {
            points3d.push_back( i->mapPoint()->point() );
            points2d.push_back( i->point() );
        }

        if ( points3d.size() < MIN_TRACK_POINTS_COUNT )
            return 0.;

        cv::Mat rvec;
        cv::Mat tvec;

        std::vector< int > inliers;

        if ( !cv::solvePnPRansac( points3d, points2d, prevFrame->cameraMatrix(), cv::noArray(), rvec, tvec, false,
                                        100, maxReprojectionError, 0.99, inliers, cv::SOLVEPNP_ITERATIVE ) )
            return 0.;

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

    return 0.;

}

void AdjacentFrame::setFrames( const FeatureFramePtr &prevFrame, const FeatureFramePtr &nextFrame )
{
    DoubleFrame::setFrames( prevFrame, nextFrame );
}

AdjacentFrame::FeatureFramePtr AdjacentFrame::previousFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( frame1() );
}

AdjacentFrame::FeatureFramePtr AdjacentFrame::nextFrame() const
{
    return std::dynamic_pointer_cast< FeatureFrame >( frame2() );
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

std::vector< DoubleFrame::MonoPointPtr > AdjacentFrame::posePoints() const
{
    return nextFrame()->posePoints();
}

int AdjacentFrame::posePointsCount() const
{
    return nextFrame()->posePointsCount();
}

std::vector< AdjacentFrame::MonoPointPtr > AdjacentFrame::trackFramePoints() const
{
    return previousFrame()->trackFramePoints();
}

int AdjacentFrame::trackFramePointsCount() const
{
    return trackFramePoints().size();
}

std::vector< DoubleFrame::MonoPointPtr > AdjacentFrame::trackedPoints() const
{
    return previousFrame()->trackedPoints();
}

int AdjacentFrame::trackedPointsCount() const
{
    return trackedPoints().size();
}

void AdjacentFrame::createFramePoints( const size_t count )
{
    previousFrame()->createFramePoints( count );
}

AdjacentFrame::MapPtr AdjacentFrame::parentMap() const
{
    return m_parentMap.lock();
}

AdjacentFrame::WorldPtr AdjacentFrame::parentWorld() const
{
    return parentMap()->parentWorld();
}

// StereoFrame
StereoFrame::StereoFrame( const MapPtr &parentMap )
    : StereoFrameBase( parentMap )
{
    initialize();
}

void StereoFrame::initialize()
{
    setFrames( Frame::create(), Frame::create() );
}

StereoFrame::StereoFramePtr StereoFrame::create( const MapPtr &parentMap )
{
    return StereoFramePtr( new StereoFrame( parentMap ) );
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

// DenseFrame
const double DenseFrame::m_maximumLenght = 40;

DenseFrame::DenseFrame( const MapPtr &parentMap )
    : StereoFrame( parentMap )
{
}

DenseFrame::FramePtr DenseFrame::create( const MapPtr &parentMap )
{
    return FramePtr( new DenseFrame( parentMap ) );
}

void DenseFrame::replace( const ProcessedDenseFramePtr &frame )
{
    StereoFrame::replace( frame );

    replaceProcedure( frame );

}

void DenseFrame::replaceAndClean( const ProcessedDenseFramePtr &frame )
{
    StereoFrame::replaceAndClean( frame );

    replaceProcedure( frame );

}

void DenseFrame::replaceProcedure( const ProcessedDenseFramePtr &frame )
{
    if ( frame ) {
        // TODO: Smarter selection
        for ( auto &i : frame->m_points )
            if ( cv::norm( i.point() ) < m_maximumLenght )
                m_points.push_back( i );

        setOptimizationGrid( frame->m_optimizationGrid );
    }

}

std::list< ColorPoint3d > DenseFrame::translatedPoints() const
{
    std::list< ColorPoint3d > ret;

    auto leftFrame = this->leftFrame();

    if ( leftFrame ) {
        cv::Mat rotation = leftFrame->rotation().t();
        cv::Mat translation = leftFrame->translation();

        for ( auto &i : m_points ) {
            auto point = i;
            point.setPoint( matToPoint3f< double >( rotation * ( point3fToMat< double >( i.point() ) - translation ) ) );
            ret.push_back( point );

        }

    }

    return ret;
}

}

