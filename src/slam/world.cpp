#include "src/common/precompiled.h"

#include "world.h"

#include "src/common/functions.h"

#include "frame.h"

namespace slam {

World::World( const StereoCameraMatrix &cameraMatrix )
{
    initialize( cameraMatrix );

    m_stereoProcessor.setDisparityToDepthMatrix( cameraMatrix.disparityToDepthMatrix() );

    m_stereoProcessor.setPreFilterSize( 15 );
    m_stereoProcessor.setPreFilterCap( 12 );
    m_stereoProcessor.setBlockSize( 11 );
    m_stereoProcessor.setMinDisparity( -128 );
    m_stereoProcessor.setNumDisparities( 256 );
    m_stereoProcessor.setTextureThreshold( 50 );
    m_stereoProcessor.setUniquenessRatio( 100 );
    m_stereoProcessor.setSpeckleWindowSize( 120 );
    m_stereoProcessor.setSpeckleRange( 10 );
    m_stereoProcessor.setDisp12MaxDiff( 0 );
}

void World::initialize( const StereoCameraMatrix &cameraMatrix )
{
    auto flowTracker = new CPUFlowTracker();
    m_flowTracker = std::unique_ptr< FlowTracker >( flowTracker );

    // m_featureTracker = std::unique_ptr< FeatureTracker >( new SiftTracker() );

    m_startCameraMatrix = cameraMatrix;
}

World::ObjectPtr World::create( const StereoCameraMatrix &cameraMatrix )
{
    return ObjectPtr( new World( cameraMatrix ) );
}

const std::list< MapPtr > &World::maps() const
{
    return m_maps;
}

bool World::track( const StampedImage &leftImage, const StampedImage &rightImage )
{
    auto restoreRotation = this->restoreRotation();
    auto restoreTranslation = this->restoreTranslation();

    if ( m_maps.empty() ) {

        auto startCameraMatrix = m_startCameraMatrix;

        if ( !restoreRotation.empty() && !restoreTranslation.empty() ) {
            startCameraMatrix.rotate( restoreRotation );
            startCameraMatrix.translate( restoreTranslation );

        }

        createMap( startCameraMatrix );

    }

    if ( !m_maps.empty() ) {

        auto result = m_maps.back()->track( leftImage, rightImage );

        if ( !result ) {

            std::cout << "\nTrack lost!\n" << std::endl ;

            StereoCameraMatrix projectionMatrix;

            if ( !restoreRotation.empty() && !restoreTranslation.empty() ) {

                projectionMatrix = m_startCameraMatrix;

                projectionMatrix.rotate( restoreRotation );
                projectionMatrix.translate( restoreTranslation );

            }
            else
                projectionMatrix = m_maps.back()->backProjectionMatrix();

            if ( m_maps.back()->isRudimental() )
                m_maps.pop_back();

            createMap( projectionMatrix );

            m_maps.back()->track( leftImage, rightImage );

        }

        return result;

    }

    return false;

}

void World::setRestoreMatrix( const cv::Mat &rotation, const cv::Mat &translation )
{
    m_restoreMatrix = cv::Mat( 3, 4, CV_64F );
    rotation.copyTo( m_restoreMatrix.rowRange( 0, 3 ).colRange( 0, 3 ) );
    translation.copyTo( m_restoreMatrix.rowRange( 0, 3 ).col( 3 ) );
}

const cv::Mat &World::restoreMatrix() const
{
    return m_restoreMatrix;
}

cv::Mat World::restoreRotation() const
{
    if ( !m_restoreMatrix.empty() )
        return m_restoreMatrix.rowRange( 0, 3 ).colRange( 0, 3 );
    else
        return cv::Mat();
}

cv::Mat World::restoreTranslation() const
{
    if ( !m_restoreMatrix.empty() )
        return m_restoreMatrix.rowRange( 0, 3 ).col( 3 );
    else
        return cv::Mat();
}

BMStereoProcessor &World::stereoProcessor()
{
    return m_stereoProcessor;
}

const BMStereoProcessor &World::stereoProcessor() const
{
    return m_stereoProcessor;
}

const std::unique_ptr< FlowTracker > &World::flowTracker() const
{
    return m_flowTracker;
}

const std::unique_ptr< FeatureTracker > &World::featureTracker() const
{
    return m_featureTracker;
}

const Settings &World::settings() const
{
    return m_settings;
}

double World::maxReprojectionError() const
{
    return m_settings.maxReprojectionError();
}

double World::minStereoDisparity() const
{
    return m_settings.minStereoDisparity();
}

double World::minAdjacentPointsDistance() const
{
    return m_settings.minAdjacentPointsDistance();
}

double World::minAdjacentCameraMultiplier() const
{
    return m_settings.minAdjacentCameraMultiplier();
}

double World::minTrackInliersRatio() const
{
    return m_settings.minTrackInliersRatio();
}

double World::goodTrackInliersRatio() const
{
    return m_settings.goodTrackInliersRatio();
}

CvImage World::pointsImage() const
{
    if ( !m_maps.empty() ) {

        auto frames = m_maps.back()->frames();

        if ( !frames.empty() ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::StereoKeyFrame >( frames.back() );

            // if ( processedFrame )
                // return processedFrame->drawExtractedPoints();

        }

    }

    return CvImage();

}

CvImage World::tracksImage() const
{
    if ( !m_maps.empty() ) {

        auto frames = m_maps.back()->frames();

        if ( !frames.empty() ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( frames.back() );

            if ( processedFrame )
                return processedFrame->leftFrame()->drawTracks();

        }

    }

    return CvImage();

}

CvImage World::stereoImage() const
{
    if ( !m_maps.empty() ) {

        auto frames = m_maps.back()->frames();

        if ( frames.size() > 1 ) {

            auto processedFrame = std::dynamic_pointer_cast< slam::ProcessedStereoFrame >( *(++frames.rbegin()) );

            if ( processedFrame )
                return processedFrame->drawStereoCorrespondences();

        }

    }

    return CvImage();

}

std::list< StereoCameraMatrix > World::path() const
{
    std::list< StereoCameraMatrix > ret;

    for ( auto &map : m_maps ) {

        auto frames = map->frames();

        for ( auto &i : frames ) {

            auto keyFrame = std::dynamic_pointer_cast< FinishedStereoKeyFrame >( i );

            if ( keyFrame )
                ret.push_back( keyFrame->projectionMatrix() );

        }

    }

    return ret;
}

std::list< ColorPoint3d > World::sparseCloud() const
{
    std::list< ColorPoint3d > ret;

    for ( auto &map : m_maps ) {

        auto mapPoints = map->mapPoints();

        for ( auto &i : mapPoints ) {

            if ( i )
                ret.push_back( ColorPoint3d( i->point(), i->color() ) );

        }

    }

    return ret;
}

void World::createMap( const StereoCameraMatrix &cameraMatrix )
{
    m_maps.push_back( Map::create( cameraMatrix, shared_from_this() ) );
}

}
