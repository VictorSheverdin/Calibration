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

void Map::track( const StampedStereoImage &image )
{
    auto frame = ProcStereoFrame::create( shared_from_this() );

    auto system = parentSystem();

    frame->setCameraMatrices( system->parameters().cameraMatrix() );
    frame->setDistorsionCoefficients( system->parameters().distorsionCoefficients() );

    frame->load( image );

    frame->prepareFrame();

    frame->extractFeatures();

    frame->matchCorners();

    _sequence.push_back( frame );
}

}
