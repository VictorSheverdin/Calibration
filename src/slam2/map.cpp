#include "src/common/precompiled.h"

#include "map.h"

#include "mappoint.h"
#include "system.h"
#include "frame.h"

namespace slam2 {

// Map
Map::Map( const SystemPtr &parent )
    : Parent_Shared_Ptr< System >( parent )
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

const std::vector< MapPointPtr > &Map::mapPoints() const
{
    return _points;
}

MapPointPtr Map::createMapPoint( const ColorPoint3d &point )
{
    auto ret = MapPoint::create( point, shared_from_this() );

    _points.push_back( ret );

    return ret;
}

ProcStereoFramePtr Map::track( const StampedStereoImage &image )
{
    auto frame = ProcStereoFrame::create( shared_from_this() );

    auto system = parentSystem();

    frame->setCameraMatrices( system->parameters().cameraMatrix() );
    frame->setDistorsionCoefficients( system->parameters().distorsionCoefficients() );

    frame->setRightRotation( system->parameters().rightRotation() );
    frame->setRightTranslation( system->parameters().rightTranslation() );

    frame->load( image );

    if ( !_sequence.empty() ) {

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

            if ( inliers < system->parameters().minimumInliersRatio() )
                return ProcStereoFramePtr();

        }

    }

    _sequence.push_back( frame );

    // TEMPORARY:
    if ( _sequence.size() > 10 )
        _sequence.pop_front();

    return frame;

}

const std::list< StereoFramePtr > &Map::sequence() const
{
    return _sequence;
}

}
