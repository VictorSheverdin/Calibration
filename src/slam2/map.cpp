#include "src/common/precompiled.h"

#include "map.h"

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

void Map::track( const StampedStereoImage &image )
{
    auto frame = ProcStereoFrame::create( shared_from_this() );

    auto system = parentSystem();

    frame->setCameraMatrices( system->parameters().cameraMatrix() );
    frame->setDistorsionCoefficients( system->parameters().distorsionCoefficients() );

    frame->setRightRotation( system->parameters().rightRotation() );
    frame->setRightTranslation( system->parameters().rightTranslation() );

    frame->load( image );

    frame->prepareFrame();

    frame->extract();

    frame->triangulatePoints();

    // TEMPORARY:
    if ( _sequence.size() > 5 )
        _sequence.clear();

    _sequence.push_back( frame );

}

}
