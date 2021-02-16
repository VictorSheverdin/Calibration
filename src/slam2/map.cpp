#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"
#include "system.h"
#include "frame.h"

namespace slam2 {

// Map
Map::Map( const SystemPtr &parent )
    : Parent_Weak_Ptr< System >( parent )
{
}

Map::ObjectPtr Map::create( const SystemPtr &parent )
{
    return ObjectPtr( new Map( parent ) );
}

std::shared_ptr< System > Map::parentSystem() const
{
    return parentPointer();
}

CvImage Map::drawPoints() const
{
    if ( _sequence.size() > 1 ) {
        auto procFrame = std::dynamic_pointer_cast< ProcStereoFrame >( *( ++_sequence.rbegin() ) );

        if ( procFrame )
            return procFrame->drawPoints();
    }

    return CvImage();
}

CvImage Map::drawTracks() const
{
    if ( !_sequence.empty() ) {
        auto procFrame = std::dynamic_pointer_cast< ProcStereoFrame >( _sequence.back() );

        if ( procFrame )
            return procFrame->drawTracks();
    }

    return CvImage();
}

CvImage Map::drawStereo() const
{
    if ( _sequence.size() > 1 ) {
        auto procFrame = std::dynamic_pointer_cast< ProcStereoFrame >( *( ++_sequence.rbegin() ) );

        if ( procFrame )
            return procFrame->drawStereo();
    }

    return CvImage();
}

std::vector< ColorPoint3d > Map::lastSparseCloud() const
{
    if ( _sequence.size() > 1 ) {
        auto stereoFrame = *( ++_sequence.rbegin() );

        if ( stereoFrame )
            return stereoFrame->sparseCloud();
    }

    return std::vector< ColorPoint3d >();

}

bool Map::track( const StampedStereoImage &image )
{
    auto frame = ProcStereoFrame::create( shared_from_this() );

    auto system = parentSystem();

    frame->setCameraMatrices( system->parameters().cameraMatrix() );
    frame->setDistorsionCoefficients( system->parameters().distorsionCoefficients() );

    frame->setRightRotation( system->parameters().rightRotation() );
    frame->setRightTranslation( system->parameters().rightTranslation() );

    frame->setMask( system->parameters().mask() );

    frame->load( image );

    if ( !_sequence.empty() ) {

        if ( _sequence.size() > 1 )
            finalizeFrame( ++_sequence.rbegin() );

        auto prevFrame = std::dynamic_pointer_cast< ProcStereoFrame >( _sequence.back() );

        if ( prevFrame ) {

            if ( prevFrame->leftFrame()->recoverPointsCount() < system->parameters().minimumTracksCount() ) {
                prevFrame->extract();
                prevFrame->match();
                prevFrame->triangulatePoints();
            }

            auto consecutiveFrame = ConsecutiveFrame::create( prevFrame->leftFrame(), frame->leftFrame(), shared_from_this() );

            consecutiveFrame->extract();
            consecutiveFrame->track();

            auto inliers = frame->recoverPose();

            qDebug() << inliers;

            frame->triangulateTracks();

            if ( inliers < system->parameters().minimumInliersRatio() ) {
                finalizeFrame( _sequence.rbegin() );
                qDebug() << "Track lost!";
                return false;
            }

        }

    }

    _sequence.push_back( frame );

    return true;

}

void Map::finalizeFrame( const std::reverse_iterator< std::list< StereoFramePtr >::iterator > it )
{
    auto frame = std::dynamic_pointer_cast< ProcStereoFrame >( *it );

    auto finalFrame = FinalStereoFrame::create( shared_from_this() );

    finalFrame->replace( frame ) ;

    *it = finalFrame;
}

const std::list< StereoFramePtr > &Map::sequence() const
{
    return _sequence;
}

StereoFramePtr Map::lastFrame() const
{
    if ( !_sequence.empty() )
        return _sequence.back();
    else
        return StereoFramePtr();
}


}
