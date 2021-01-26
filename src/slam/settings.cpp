#include "src/common/precompiled.h"

#include "settings.h"

namespace slam {

double Settings::m_maxReprojectionError = 1.;
double Settings::m_minStereoDisparity = 7.;
double Settings::m_minAdjacentPointsDistance = 7.;
double Settings::m_minAdjacentCameraMultiplier = 1.;

double Settings::m_minTrackInliersRatio = 0.7;
double Settings::m_goodTrackInliersRatio = 0.9;

double Settings::maxReprojectionError() const
{
    return m_maxReprojectionError;
}

double Settings::minStereoDisparity() const
{
    return m_minStereoDisparity;
}

double Settings::minAdjacentPointsDistance() const
{
    return m_minAdjacentPointsDistance;
}

double Settings::minAdjacentCameraMultiplier() const
{
    return m_minAdjacentCameraMultiplier;
}

double Settings::minTrackInliersRatio() const
{
    return m_minTrackInliersRatio;
}

double Settings::goodTrackInliersRatio() const
{
    return m_goodTrackInliersRatio;
}

}
