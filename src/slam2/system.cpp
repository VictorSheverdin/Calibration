#include "src/common/precompiled.h"

#include "system.h"

#include "map.h"

namespace slam2 {

// System
System::System( const Parameters &parameters )
{
    initialize();

    setParameters( parameters );
}

void System::initialize()
{
}

System::ObjectPtr System::create( const Parameters &parameters )
{
    return ObjectPtr( new System( parameters ) );
}

void System::createMap()
{
    _maps.push_back( Map::create( shared_from_this() ) );
}

void System::setParameters( const Parameters &value )
{
    _parameters = value;
}

const Parameters &System::parameters() const
{
    return _parameters;
}

const std::shared_ptr< Tracker > &System::tracker() const
{
    return _parameters.tracker();
}

void System::track( const StampedStereoImage &image )
{
    CV_Assert( !_maps.empty() );

    CV_Assert( _maps.back() );

    _maps.back()->track( image );
}

}
