#pragma once

#include <vector>
#include <map>
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

    static ObjectPtr create();

    void addPoint( const Point2Ptr &point );

    std::vector< Point2Ptr > validPoints() const;

protected:
    Track();

    std::map< size_t, Point2Weak > _points;

    MapPointWeak _mapPoint;

    size_t _maxIndex;

private:
    void initialize();
};

}
