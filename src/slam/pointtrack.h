#include <vector>
#include <list>
#include <memory>

namespace slam {

class MonoPoint;
class WorldPoint;

class PointTrack : std::enable_shared_from_this< PointTrack >
{
public:
    using TrackPtr = std::shared_ptr< PointTrack >;

    using WorldPointPtr = std::weak_ptr< WorldPoint >;
    using MonoPointPtr = std::shared_ptr< MonoPoint >;

    static TrackPtr create();
    static TrackPtr create( const WorldPointPtr worldPoint );

    void setWorldPoint( const WorldPointPtr value );

    std::list< MonoPointPtr > &points();
    const std::list< MonoPointPtr > &points() const;

    void addPoint( const MonoPointPtr point );

protected:
    PointTrack();
    PointTrack( const WorldPointPtr worldPoint );

    WorldPointPtr m_parentWorldPoint;

    std::list< MonoPointPtr > m_points;

private:
};

}
