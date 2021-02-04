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
    for ( auto i = _sequence.rbegin(); i!= _sequence.rend(); ++i ) {
        auto procFrame = std::dynamic_pointer_cast< ProcStereoFrame >( *i );
        if ( procFrame )
            return procFrame->drawPoints();
    }

    return CvImage();
}

CvImage Map::drawTracks() const
{
    for ( auto i = _sequence.rbegin(); i!= _sequence.rend(); ++i ) {
        auto procFrame = std::dynamic_pointer_cast< ProcStereoFrame >( *i );
        if ( procFrame )
            return procFrame->drawTracks();
    }

    return CvImage();
}

CvImage Map::drawStereo() const
{
    for ( auto i = _sequence.rbegin(); i!= _sequence.rend(); ++i ) {
        auto procFrame = std::dynamic_pointer_cast< ProcStereoFrame >( *i );
        if ( procFrame )
            return procFrame->drawStereo();
    }

    return CvImage();
}

std::vector< ColorPoint3d > Map::lastSparseCloud() const
{
    for ( auto i = _sequence.rbegin(); i!= _sequence.rend(); ++i ) {
        auto procFrame = std::dynamic_pointer_cast< ProcStereoFrame >( *i );
        if ( procFrame )
            return procFrame->sparseCloud();
    }

    return std::vector< ColorPoint3d >();
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

void Map::track( const StampedStereoImage &image )
{
    auto frame = ProcStereoFrame::create( shared_from_this() );

    auto system = parentSystem();

    frame->setCameraMatrices( system->parameters().cameraMatrix() );
    frame->setDistorsionCoefficients( system->parameters().distorsionCoefficients() );

    frame->setRightRotation( system->parameters().rightRotation() );
    frame->setRightTranslation( system->parameters().rightTranslation() );

    frame->load( image );

    if ( _sequence.empty() ) {
        frame->prepareFrame();
        frame->extract();
        frame->match();
        frame->triangulatePoints();
    }
    else {
        auto prevFrame = std::dynamic_pointer_cast< ProcStereoFrame >( _sequence.back() );

        auto consecutiveFrames = ConsecutiveFrames::create( prevFrame->leftFrame(), frame->leftFrame(), shared_from_this() );

        consecutiveFrames->prepareFrame();
        consecutiveFrames->extract();
        consecutiveFrames->track();

        if ( frame->leftFrame()->stereoTracksCount() < system->parameters().minimumTracksCount() ) {
            frame->prepareFrame();
            frame->extract();
            frame->match();
            frame->triangulatePoints();
        }

        // TEMPORARY:
        prevFrame->clearMemory();

    }

    _sequence.push_back( frame );

    // TEMPORARY:
    if ( _sequence.size() > 100 )
        _sequence.pop_front();

}

}
