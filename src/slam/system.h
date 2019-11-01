#pragma once

#include "src/common/calibrationdatabase.h"
#include "src/common/rectificationprocessor.h"

#include "map.h"

namespace slam {

class System
{
public:
    System( const StereoCalibrationDataShort &calibration );
    System( const std::string &calibrationFile );

    bool track( const std::string &leftFile, const std::string &rightFile );
    bool track( const CvImage &leftImage, const CvImage &rightImage );

protected:
    StereoRectificationProcessor m_rectificationProcessor;

    Map m_map;

private:
    void initialize();
};

}
