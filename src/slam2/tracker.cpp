#include "src/common/precompiled.h"

#include "tracker.h"

#include "system.h"
#include "map.h"
#include "track.h"
#include "frame.h"
#include "framepoint.h"
#include "mappoint.h"

namespace slam2 {

// Tracker
void Tracker::track( ConsecutiveStereoFrame &frames )
{
    auto prevFrame = frames.prevFrame();
    auto nextFrame = frames.nextFrame();
}

// FlowTracker
FlowTracker::FlowTracker()
{
    initialize();
}

void FlowTracker::initialize()
{
}

double FlowTracker::extractPrecision() const
{
    return _flowProcessor->extractPrecision();
}

void FlowTracker::setExtractPrecision( const double value )
{
    _flowProcessor->setExtractPrecision( value );
}

void FlowTracker::setExtractionDistance( const double value )
{
    _flowProcessor->setExtractionDistance( value );
}

double FlowTracker::extractionDistance() const
{
    return _flowProcessor->extractionDistance();
}

size_t FlowTracker::winSize() const
{
    return _flowProcessor->winSize();
}

void FlowTracker::setWinSize( const size_t value )
{
    _flowProcessor->setWinSize( value );
}

size_t FlowTracker::levels() const
{
    return _flowProcessor->levels();
}

void FlowTracker::setLevels( const size_t value )
{
    _flowProcessor->setLevels( value );
}

void FlowTracker::setRansacReprojectionThreshold( const double &value )
{
    _flowProcessor->setRansacReprojectionThreshold( value );
}

double FlowTracker::ransacReprojectionThreshold() const
{
    return _flowProcessor->ransacReprojectionThreshold();
}

void FlowTracker::setRansacConfidence( const double &value )
{
    _flowProcessor->setRansacConfidence( value );
}

double FlowTracker::ransacConfidence() const
{
    return _flowProcessor->ransacConfidence();
}

void FlowTracker::extractPoints( ProcFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    auto count = std::max( static_cast< int >( frame->extractionCornersCount() ) - static_cast< int >( frame->cornerPoints().size() ), 0 );

    _flowProcessor->extractPoints( frame->image(), frame->mask(), &cornerPoints, count );

    frame->addCornerPoints( cornerPoints );

}

void FlowTracker::track( ConsecutiveStereoFrame &frames )
{
    auto prevFrame = frames.prevFrame();
    auto nextFrame = frames.nextFrame();
}

// CPUFlowTracker
CPUFlowTracker::CPUFlowTracker()
{
    initialize();
}

void CPUFlowTracker::initialize()
{
    _flowProcessor = std::unique_ptr< FlowProcessor >( new CPUFlowProcessor() );
}

void CPUFlowTracker::prepareFrame( ProcFrame *frame )
{
    if ( frame->imagePyramid().empty() ) {
        std::vector< cv::Mat > pyramid;
        processor()->buildImagePyramid( frame->image(), &pyramid );
        frame->setImagePyramid( pyramid );

    }

}

void CPUFlowTracker::extract( ProcStereoFrame *frame )
{
    extractPoints( frame->leftFrame().get() );
}

void CPUFlowTracker::extract( ConsecutiveFrame *frame )
{
    extractPoints( frame->frame1().get() );
}

void CPUFlowTracker::match( ProcStereoFrame *frame )
{
    auto system = frame->parentSystem();

    std::vector< FlowTrackResult > trackResults;

    auto leftFrame = frame->leftFrame();
    auto rightFrame = frame->rightFrame();

    prepareFrame( leftFrame.get() );
    prepareFrame( rightFrame.get() );

    processor()->track( leftFrame->imagePyramid(), leftFrame->cornerPoints(), rightFrame->imagePyramid(), &trackResults );

    std::vector< size_t > indexes;

    for ( auto &i : trackResults )
        indexes.push_back( frame->rightFrame()->addCornerPoint( i ) );

    std::vector< cv::Point2f > leftPoints;
    std::vector< cv::Point2f > rightPoints;

    for ( size_t i = 0; i < trackResults.size(); ++i ) {
        leftPoints.push_back( leftFrame->undistortedCornerPoint( trackResults[ i ].index ) );
        rightPoints.push_back( rightFrame->undistortedCornerPoint( indexes[ i ] ) );
    }

    std::vector< size_t > inliers;

    processor()->epipolarTest( leftPoints, rightPoints, &inliers );

    for ( auto &i : inliers )
        frame->createFlowPoint( trackResults[ i ].index, indexes[ i ] );

}

void CPUFlowTracker::match( ConsecutiveFrame *frame )
{
    auto frame1 = frame->frame1();
    auto frame2 = frame->frame2();

    prepareFrame( frame1.get() );
    prepareFrame( frame2.get() );

    std::vector< FlowTrackResult > trackResults;

    processor()->track( frame1->imagePyramid(), frame1->cornerPoints(), frame2->imagePyramid(), &trackResults );

    std::vector< size_t > indexes;

    for ( auto &i : trackResults )
        indexes.push_back( frame2->addCornerPoint( i ) );

    std::vector< cv::Point2f > points1;
    std::vector< cv::Point2f > points2;

    for ( size_t i = 0; i < trackResults.size(); ++i ) {
        points1.push_back( frame1->undistortedCornerPoint( trackResults[ i ].index ) );
        points2.push_back( frame2->undistortedCornerPoint( indexes[ i ] ) );
    }

    std::vector< size_t > inliers;

    processor()->epipolarTest( points1, points2, &inliers );

    auto map = frame->parentMap();

    for ( auto &i : inliers ) {
        auto flowPoint = frame2->createFlowPoint( indexes[ i ]  );

        auto prevFlowPoint = frame1->flowPoint( trackResults[ i ].index );

        if ( prevFlowPoint ) {

            auto track = prevFlowPoint->parentTrack();

            if ( !track ) {
                track = frame1->createTrack();
                track->addPoint( prevFlowPoint );

            }

            track->addPoint( flowPoint );

            auto stereoPoint = prevFlowPoint->stereoPoint();

            if ( stereoPoint ) {

                if ( stereoPoint->isPoint3dExist() ) {

                    auto mapPoint = track->mapPoint();

                    if ( !mapPoint )
                        track->createMapPoint( ColorPoint3d( stereoPoint->point3d(), stereoPoint->color() ) );
                    else
                        mapPoint->setPoint( ColorPoint3d( stereoPoint->point3d(), stereoPoint->color() ) );

                }

            }


        }

    }

}

CPUFlowProcessor *CPUFlowTracker::processor() const
{
    return dynamic_cast< CPUFlowProcessor* >( _flowProcessor.get() );
}

// GPUFlowTracker
GPUFlowTracker::GPUFlowTracker()
{
    initialize();
}

void GPUFlowTracker::initialize()
{
    _flowProcessor = std::unique_ptr< FlowProcessor >( new GPUFlowProcessor() );
}

void GPUFlowTracker::extract( ProcStereoFrame *frame )
{
    extractPoints( frame->leftFrame().get() );
}

void GPUFlowTracker::extract( ConsecutiveFrame *frame )
{
    extractPoints( frame->frame1().get() );
}

void GPUFlowTracker::match( ProcStereoFrame *frame )
{
    auto system = frame->parentSystem();

    std::vector< FlowTrackResult > trackResults;

    auto leftFrame = frame->leftFrame();
    auto rightFrame = frame->rightFrame();

    processor()->track( leftFrame->image(), leftFrame->cornerPoints(), rightFrame->image(), &trackResults );

    std::vector< size_t > indexes;

    for ( auto &i : trackResults )
        indexes.push_back( frame->rightFrame()->addCornerPoint( i ) );

    std::vector< cv::Point2f > leftPoints;
    std::vector< cv::Point2f > rightPoints;

    for ( size_t i = 0; i < trackResults.size(); ++i ) {
        leftPoints.push_back( leftFrame->undistortedCornerPoint( trackResults[ i ].index ) );
        rightPoints.push_back( rightFrame->undistortedCornerPoint( indexes[ i ] ) );
    }

    std::vector< size_t > inliers;

    processor()->epipolarTest( leftPoints, rightPoints, &inliers );

    for ( auto &i : inliers )
        frame->createFlowPoint( trackResults[ i ].index, indexes[ i ] );

}

void GPUFlowTracker::match( ConsecutiveFrame *frame )
{
    auto frame1 = frame->frame1();
    auto frame2 = frame->frame2();

    std::vector< FlowTrackResult > trackResults;

    processor()->track( frame1->image(), frame1->cornerPoints(), frame2->image(), &trackResults );

    std::vector< size_t > indexes;

    for ( auto &i : trackResults )
        indexes.push_back( frame2->addCornerPoint( i ) );

    std::vector< cv::Point2f > points1;
    std::vector< cv::Point2f > points2;

    for ( size_t i = 0; i < trackResults.size(); ++i ) {
        points1.push_back( frame1->undistortedCornerPoint( trackResults[ i ].index ) );
        points2.push_back( frame2->undistortedCornerPoint( indexes[ i ] ) );
    }

    std::vector< size_t > inliers;

    processor()->epipolarTest( points1, points2, &inliers );

    auto map = frame->parentMap();

    for ( auto &i : inliers ) {

        auto flowPoint = frame2->createFlowPoint( indexes[ i ]  );

        auto prevFlowPoint = frame1->flowPoint( trackResults[ i ].index );

        if ( prevFlowPoint ) {

            auto track = prevFlowPoint->parentTrack();

            if ( !track ) {
                track = frame1->createTrack();
                track->addPoint( prevFlowPoint );

            }

            track->addPoint( flowPoint );

            auto stereoPoint = prevFlowPoint->stereoPoint();

            if ( stereoPoint ) {

                if ( stereoPoint->isPoint3dExist() ) {

                    auto mapPoint = track->mapPoint();

                    if ( !mapPoint )
                        track->createMapPoint( ColorPoint3d( stereoPoint->point3d(), stereoPoint->color() ) );
                    else
                        mapPoint->setPoint( ColorPoint3d( stereoPoint->point3d(), stereoPoint->color() ) );

                }

            }


        }

    }

}

GPUFlowProcessor *GPUFlowTracker::processor() const
{
    return dynamic_cast< GPUFlowProcessor* >( _flowProcessor.get() );
}

// FeatureTrackerAndMatcher
void FeatureTrackerAndMatcher::extract( ProcFrame *frame )
{
    std::vector< cv::Point2f > cornerPoints;

    std::vector< cv::KeyPoint > keypoints;
    cv::Mat descriptors;

    _descriptorProcessor->extractAndCompute( frame->image(), frame->mask(), &keypoints, &descriptors );

    frame->setKeyPoints( keypoints );
    frame->setDescriptors( descriptors );
}

void FeatureTrackerAndMatcher::setRansacReprojectionThreshold( const double &value )
{
    _descriptorProcessor->setRansacReprojectionThreshold( value );
}

double FeatureTrackerAndMatcher::ransacReprojectionThreshold() const
{
    return _descriptorProcessor->ransacReprojectionThreshold();
}

void FeatureTrackerAndMatcher::setRansacConfidence( const double &value )
{
    _descriptorProcessor->setRansacConfidence( value );
}

double FeatureTrackerAndMatcher::ransacConfidence() const
{
    return _descriptorProcessor->ransacConfidence();
}

void FeatureTrackerAndMatcher::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
    extract( frame->rightFrame().get() );
}

void FeatureTrackerAndMatcher::extract( ConsecutiveFrame *frame )
{
    extract( frame->frame2().get() );
}

void FeatureTrackerAndMatcher::match( ProcStereoFrame *frame )
{
    std::vector< cv::DMatch > matches;

    auto leftFrame = frame->leftFrame();
    auto rightFrame = frame->rightFrame();

    _featuresMatcher->match( leftFrame->keyPoints(), leftFrame->descriptors(), rightFrame->keyPoints(), rightFrame->descriptors(), &matches );

    std::vector< cv::Point2f > leftPoints;
    std::vector< cv::Point2f > rightPoints;

    leftPoints.reserve( matches.size() );
    rightPoints.reserve( matches.size() );

    for ( auto &i : matches ) {
        leftPoints.push_back( leftFrame->undistortedKeyPoint( i.queryIdx ).pt );
        rightPoints.push_back( rightFrame->undistortedKeyPoint( i.trainIdx ).pt );
    }

    std::vector< size_t > inliers;

    _descriptorProcessor->epipolarTest( leftPoints, rightPoints, &inliers );

    for ( auto &i : inliers )
        frame->createFeaturePoint( matches[ i ].queryIdx, matches[ i ].trainIdx );
}

void FeatureTrackerAndMatcher::match( ConsecutiveFrame *frame )
{
    auto frame1 = frame->frame1();
    auto frame2 = frame->frame2();

    std::vector< cv::DMatch > matches;

    _featuresMatcher->match( frame1->keyPoints(), frame1->descriptors(), frame2->keyPoints(), frame2->descriptors(), &matches );

    std::vector< cv::Point2f > points1;
    std::vector< cv::Point2f > points2;

    points1.reserve( matches.size() );
    points2.reserve( matches.size() );

    for ( auto &i : matches ) {
        points1.push_back( frame1->undistortedKeyPoint( i.queryIdx ).pt );
        points2.push_back( frame2->undistortedKeyPoint( i.trainIdx ).pt );
    }

    std::vector< size_t > inliers;

    _descriptorProcessor->epipolarTest( points1, points2, &inliers );

    auto map = frame->parentMap();

    for ( auto &i : inliers ) {

        auto featurePoint = frame2->createFeaturePoint( matches[ i ].trainIdx );

        auto prevFeaturePoint = frame1->featurePoint( matches[ i ].queryIdx );

        if ( prevFeaturePoint ) {

            auto track = prevFeaturePoint->parentTrack();

            if ( !track ) {
                track = frame1->createTrack();
                track->addPoint( prevFeaturePoint );

                auto stereoPoint = prevFeaturePoint->stereoPoint();

                if ( stereoPoint ) {

                    if ( stereoPoint->isPoint3dExist() )
                        track->createMapPoint( ColorPoint3d( stereoPoint->point3d(), stereoPoint->color() ) );


                }

            }

            track->addPoint( featurePoint );

        }

    }

}

void FeatureTrackerAndMatcher::track( ConsecutiveStereoFrame &frames )
{

}

// SiftTracker
SiftTracker::SiftTracker()
{
    initialize();
}

void SiftTracker::initialize()
{
    _descriptorProcessor = std::unique_ptr< FullProcessor >( new SiftProcessor() );
    _featuresMatcher = std::unique_ptr< DescriptorMatcher >( new FlannMatcher() );
}

SiftProcessor *SiftTracker::processor() const
{
    return dynamic_cast< SiftProcessor* >( _descriptorProcessor.get() );
}

FlannMatcher *SiftTracker::matcher() const
{
    return dynamic_cast< FlannMatcher* >( _featuresMatcher.get() );
}

// OrbTracker
OrbTracker::OrbTracker()
{
    initialize();
}

void OrbTracker::initialize()
{
    _descriptorProcessor = std::unique_ptr< FullProcessor >( new OrbProcessor() );
    _featuresMatcher = std::unique_ptr< DescriptorMatcher >( new BFMatcher() );
}

OrbProcessor *OrbTracker::processor() const
{
    return dynamic_cast< OrbProcessor* >( _descriptorProcessor.get() );
}

BFMatcher *OrbTracker::matcher() const
{
    return dynamic_cast< BFMatcher* >( _featuresMatcher.get() );
}

// AKazeTracker
AKazeTracker::AKazeTracker()
{
    initialize();
}

void AKazeTracker::initialize()
{
    _descriptorProcessor = std::make_unique< AKazeProcessor >();
    _featuresMatcher = std::make_unique< BFMatcher >();
}

AKazeProcessor *AKazeTracker::processor() const
{
    return dynamic_cast< AKazeProcessor* >( _descriptorProcessor.get() );
}

BFMatcher *AKazeTracker::matcher() const
{
    return dynamic_cast< BFMatcher* >( _featuresMatcher.get() );
}

// SuperGlueTracker
SuperGlueTracker::SuperGlueTracker()
{
    initialize();
}

void SuperGlueTracker::initialize()
{
    _processor = std::make_unique< SuperGlueProcessor >( "superpoint_fp32_2048.eng", "superglue_fp32.eng" );
}

void SuperGlueTracker::extract( ProcFrame *frame )
{
    if ( frame->keyPoints().empty() ) {
        std::vector< cv::KeyPoint > keypoints;
        cv::Mat descriptors;

        CvImage gray;

        cv::cvtColor( frame->image(), gray, cv::COLOR_BGR2GRAY );

        _processor->extract( gray, frame->mask(), &keypoints, &descriptors );

        frame->setKeyPoints( keypoints );
        frame->setDescriptors( descriptors );

    }

}

void SuperGlueTracker::setRansacReprojectionThreshold( const double &value )
{
    _processor->setRansacReprojectionThreshold( value );
}

double SuperGlueTracker::ransacReprojectionThreshold() const
{
    return _processor->ransacReprojectionThreshold();
}

void SuperGlueTracker::setRansacConfidence( const double &value )
{
    _processor->setRansacConfidence( value );
}

double SuperGlueTracker::ransacConfidence() const
{
    return _processor->ransacConfidence();
}

void SuperGlueTracker::extract( ProcStereoFrame *frame )
{
    extract( frame->leftFrame().get() );
    extract( frame->rightFrame().get() );
}

void SuperGlueTracker::extract( ConsecutiveFrame *frame )
{
    extract( frame->frame2().get() );
}

void SuperGlueTracker::match( ProcStereoFrame *frame )
{
    std::vector< cv::DMatch > matches;

    auto leftFrame = frame->leftFrame();
    auto rightFrame = frame->rightFrame();

    _processor->match( leftFrame->image().size(), rightFrame->image().size(), leftFrame->keyPoints(), rightFrame->keyPoints(), leftFrame->descriptors(), rightFrame->descriptors(), &matches );

    std::vector< cv::Point2f > leftPoints;
    std::vector< cv::Point2f > rightPoints;

    leftPoints.reserve( matches.size() );
    rightPoints.reserve( matches.size() );

    for ( auto &i : matches ) {
        leftPoints.push_back( leftFrame->undistortedKeyPoint( i.queryIdx ).pt );
        rightPoints.push_back( rightFrame->undistortedKeyPoint( i.trainIdx ).pt );
    }

    std::vector< size_t > inliers;

    _processor->epipolarTest( leftPoints, rightPoints, &inliers );

    for ( auto &i : inliers )
        frame->createFeaturePoint( matches[ i ].queryIdx, matches[ i ].trainIdx );
}

void SuperGlueTracker::match( ConsecutiveFrame *frame )
{
    auto frame1 = frame->frame1();
    auto frame2 = frame->frame2();

    std::vector< cv::DMatch > matches;

    processor()->match( frame1->image().size(), frame2->image().size(), frame1->keyPoints(), frame2->keyPoints(), frame1->descriptors(), frame2->descriptors(), &matches );

    std::vector< cv::Point2f > points1;
    std::vector< cv::Point2f > points2;

    points1.reserve( matches.size() );
    points2.reserve( matches.size() );

    for ( auto &i : matches ) {
        points1.push_back( frame1->undistortedKeyPoint( i.queryIdx ).pt );
        points2.push_back( frame2->undistortedKeyPoint( i.trainIdx ).pt );
    }

    std::vector< size_t > inliers;

    _processor->epipolarTest( points1, points2, &inliers );

    auto map = frame->parentMap();

    for ( auto &i : inliers ) {

        auto featurePoint = frame2->createFeaturePoint( matches[ i ].trainIdx );

        auto prevFeaturePoint = frame1->featurePoint( matches[ i ].queryIdx );

        if ( prevFeaturePoint ) {

            auto track = prevFeaturePoint->parentTrack();

            if ( !track ) {
                track = frame1->createTrack();
                track->addPoint( prevFeaturePoint );

                auto stereoPoint = prevFeaturePoint->stereoPoint();

                if ( stereoPoint ) {

                    if ( stereoPoint->isPoint3dExist() ) {

                        auto mapPoint = track->mapPoint();

                        if ( !mapPoint )
                            track->createMapPoint( ColorPoint3d( stereoPoint->point3d(), stereoPoint->color() ) );
                        else
                            mapPoint->setPoint( ColorPoint3d( stereoPoint->point3d(), stereoPoint->color() ) );

                    }


                }

            }

            track->addPoint( featurePoint );

        }

    }

}

void SuperGlueTracker::track( ConsecutiveStereoFrame &frames )
{
}

SuperGlueProcessor *SuperGlueTracker::processor() const
{
    return _processor.get();
}


}
