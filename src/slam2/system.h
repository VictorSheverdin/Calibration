#pragma once

#include <vector>
#include <memory>

#include "alias.h"

#include "parameters.h"

#include "src/common/image.h"

namespace slam2 {

class System : public std::enable_shared_from_this< System >
{
public:
    using ObjectClass = System;
    using ObjectPtr = std::shared_ptr< System >;
    using ObjectConstPtr = std::shared_ptr< const System >;

    virtual ~System() = default;

    static ObjectPtr create( const Parameters &parameters );

    void createMap();

    void setParameters( const Parameters &value );
    const Parameters &parameters() const;

    const std::shared_ptr< Tracker > &tracker() const;

    void track( const StampedStereoImage &image );

protected:
    System( const Parameters &parameters );

    std::vector< MapPtr > _maps;

    Parameters _parameters;

private:
    void initialize();

};

}
