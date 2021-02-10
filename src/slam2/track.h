#pragma once

#include <vector>
#include <map>
#include <memory>

#include "alias.h"

#include "src/common/colorpoint.h"

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

    void setMapPoint( const MapPointPtr &value );
    MapPointPtr mapPoint() const;

    void createMapPoint( const ColorPoint3d &point );

    void clearMapPoint();

protected:
    Track();

    std::map< size_t, Point2Weak > _points;

    MapPointPtr _mapPoint;

    size_t _maxIndex;

private:
    void initialize();
};

}
