#pragma once

#include "src/common/calibrationdatabase.h"
#include "src/common/rectificationprocessor.h"

#include "world.h"

namespace slam {

class System
{
public:
    using WorldPtr = World::WorldPtr;

    System( const StereoCalibrationDataShort &calibration );
    System( const std::string &calibrationFile );

    bool track( const std::string &leftFile, const std::string &rightFile );
    bool track( const CvImage &leftImage, const CvImage &rightImage );

protected:
    StereoRectificationProcessor m_rectificationProcessor;

    WorldPtr m_world;

private:
    void initialize();
};

}
