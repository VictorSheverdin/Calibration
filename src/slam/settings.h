#pragma once

namespace slam {

class Settings
{
public:

    double maxReprojectionError() const;

    double minStereoDisparity() const;

    double minAdjacentPointsDistance() const;
    double minAdjacentCameraMultiplier() const;

    double minTrackInliersRatio() const;
    double goodTrackInliersRatio() const;

    static double m_maxReprojectionError;

    static double m_minStereoDisparity;

    static double m_minAdjacentPointsDistance;
    static double m_minAdjacentCameraMultiplier;

    static double m_minTrackInliersRatio;
    static double m_goodTrackInliersRatio;

};

}
