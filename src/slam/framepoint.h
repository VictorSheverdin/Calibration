#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

class MonoFrame;
class WorldPoint;

class FramePointBase
{
public:
protected:
    FramePointBase();

private:

};

class MonoFramePoint : public FramePointBase
{
    friend class MonoFrame;

public:
    const cv::Point2f &point() const;

protected:
    MonoFramePoint( const MonoFrame *parentFrame, const size_t keyPointIndex );

    const MonoFrame *m_parentFrame; // Parent frame
    size_t m_keyPointIndex; // Index of keypoint in parent frame

    WorldPoint *m_parentWorldPoint; // parent world point

private:
};

class StereoFramePoint : public FramePointBase
{
    friend class StereoFrame;

public:
    const cv::Point2f &leftPoint() const;
    const cv::Point2f &rightPoint() const;

protected:
    StereoFramePoint( const std::shared_ptr< MonoFramePoint > &leftPoint, const std::shared_ptr< MonoFramePoint > &rightPoint );

    std::shared_ptr< MonoFramePoint > m_leftFramePoint;
    std::shared_ptr< MonoFramePoint > m_rightFramePoint;

private:
};

}
