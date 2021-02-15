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
    void setPoint( const size_t index, const Point2Ptr &point );

    Point2Ptr startPoint() const;

    std::vector< Point2Ptr > pointsVector() const;
    const std::map< size_t, Point2Ptr > &points() const;

    void setMapPoint( const MapPointPtr &value );
    MapPointPtr mapPoint() const;

    void createMapPoint( const ColorPoint3d &point );

    void clearMapPoint();

protected:
    Track();

    std::map< size_t, Point2Ptr > _points;

    MapPointPtr _mapPoint;

    size_t _maxIndex;

private:
    void initialize();
};

}
