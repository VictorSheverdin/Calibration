#pragma once

#include <memory>

namespace slam2 {

class MapPoint : public std::enable_shared_from_this< MapPoint >
{
public:
    using ObjectClass = MapPoint;
    using ObjectPtr = std::shared_ptr< MapPoint >;
    using ObjectConstPtr = std::shared_ptr< const MapPoint >;

protected:
    MapPoint() = default;
    virtual ~MapPoint() = default;

};

}
