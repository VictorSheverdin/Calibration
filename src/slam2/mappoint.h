#pragma once

#include <memory>

#include "src/common/supportclasses.h"

#include "src/common/colorpoint.h"

#include "alias.h"

namespace slam2 {

class MapPoint : public std::enable_shared_from_this< MapPoint >, protected Parent_Shared_Ptr< Map >
{
    friend class Map;
public:
    using ObjectClass = MapPoint;
    using ObjectPtr = std::shared_ptr< MapPoint >;
    using ObjectConstPtr = std::shared_ptr< const MapPoint >;

    virtual ~MapPoint() = default;

    void setPoint( const ColorPoint3d &point );
    const ColorPoint3d &point() const;

protected:
    MapPoint( const ColorPoint3d &point, const MapPtr &map );

    static ObjectPtr create( const ColorPoint3d &point, const MapPtr &parent );

    ColorPoint3d _point;

};

}
