#pragma once

#include <vector>
#include <memory>

#include "alias.h"

namespace slam2 {

class Track : public std::enable_shared_from_this< Track >
{
public:
    using ObjectClass = Track;
    using ObjectPtr = std::shared_ptr< Track >;
    using ObjectConstPtr = std::shared_ptr< const Track >;

    virtual ~Track() = default;

protected:
    using PointPtrImpl = std::weak_ptr< Point2 >;
    using MapPointPtrImpl = std::weak_ptr< MapPoint >;

    Track() = default;

    std::vector< PointPtrImpl > _points;

    MapPointPtrImpl _mapPoint;

};

}
