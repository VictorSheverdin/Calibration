#pragma once

#include <vector>
#include <memory>

#include "src/common/supportclasses.h"

#include "src/common/colorpoint.h"
#include "src/common/image.h"

#include "alias.h"

namespace slam2 {

class Map : public std::enable_shared_from_this< Map >, protected Parent_Shared_Ptr< System >
{
public:
    using ObjectClass = Map;
    using ObjectPtr = std::shared_ptr< Map >;
    using ObjectConstPtr = std::shared_ptr< const Map >;

    virtual ~Map() = default;

    static ObjectPtr create( const SystemPtr &parent );

    std::shared_ptr< System > parentSystem() const;

    CvImage drawPoints() const;
    CvImage drawTracks() const;
    CvImage drawStereo() const;

    MapPointPtr createMapPoint( const ColorPoint3d &point );
    const std::vector< MapPointPtr > &mapPoints() const;

    ProcStereoFramePtr track( const StampedStereoImage &image );

    const std::list< StereoFramePtr > &sequence() const;

protected:
    Map( const SystemPtr &parent );

    std::list< StereoFramePtr > _sequence;

    std::vector< MapPointPtr > _points;

};

}
