#pragma once

#include <list>

#include "frame.h"

namespace slam {

class Map
{
public:
    Map();

protected:
    std::list< FrameBase * > m_frames;

    std::vector< WorldPoint * > m_worldPoints;

private:
};

}
