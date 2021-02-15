#pragma once

#include <memory>

#include "src/common/supportclasses.h"
#include "src/common/colorpoint.h"

#include "alias.h"

namespace slam2 {

class MapPoint : public std::enable_shared_from_this< MapPoint >, protected Parent_Weak_Ptr< Track >
{
    friend class Track;
public:
    using ObjectClass = MapPoint;
    using ObjectPtr = std::shared_ptr< MapPoint >;
    using ObjectConstPtr = std::shared_ptr< const MapPoint >;

    virtual ~MapPoint() = default;

    void setPoint( const ColorPoint3d &point );
    const ColorPoint3d &point() const;

protected:
    MapPoint( const ColorPoint3d &point, const TrackPtr &parent );

    static ObjectPtr create( const ColorPoint3d &point, const TrackPtr &parent );

    ColorPoint3d _point;

};

}
