#include "src/common/precompiled.h"

#include "system.h"

#include "map.h"

#include "frame.h"

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

CvImage System::pointsImage() const
{
    if ( !_maps.empty() )
        return _maps.back()->drawPoints();
    else
        return CvImage();}

CvImage System::tracksImage() const
{
    if ( !_maps.empty() )
        return _maps.back()->drawTracks();
    else
        return CvImage();
}

CvImage System::stereoImage() const
{
    if ( !_maps.empty() )
        return _maps.back()->drawStereo();
    else
        return CvImage();
}

ProcStereoFramePtr System::track( const StampedStereoImage &image )
{
    auto procFrame = _maps.back()->track( image );

    if ( !procFrame ) {
        auto lastFrame = _maps.back()->sequence().back();

        auto newMap = Map::create( shared_from_this() );
        _maps.push_back( newMap );

        procFrame = _maps.back()->track( image );

        procFrame->setTranslation( lastFrame->translation() );
        procFrame->setRotation( lastFrame->rotation() );

    }

    return procFrame;
}

}
