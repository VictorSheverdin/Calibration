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

class DoubleFramePointBase : public FramePointBase
{
public:

protected:
    DoubleFramePointBase( const std::shared_ptr< MonoFramePoint > &point1, const std::shared_ptr< MonoFramePoint > &point2 );

    std::shared_ptr< MonoFramePoint > m_framePoint1;
    std::shared_ptr< MonoFramePoint > m_framePoint2;

};

class StereoFramePoint : public DoubleFramePointBase
{
    friend class StereoFrame;

public:
    const cv::Point2f &leftPoint() const;
    const cv::Point2f &rightPoint() const;

protected:
    StereoFramePoint( const std::shared_ptr< MonoFramePoint > &leftPoint, const std::shared_ptr< MonoFramePoint > &rightPoint );

    std::shared_ptr< MonoFramePoint > &leftFramePoint();
    std::shared_ptr< MonoFramePoint > &rightFramePoint();

    const std::shared_ptr< MonoFramePoint > &leftFramePoint() const;
    const std::shared_ptr< MonoFramePoint > &rightFramePoint() const ;

private:
};

class ConsecutiveFramePoint : public DoubleFramePointBase
{
    friend class ConsecutiveFrame;

public:

protected:
    ConsecutiveFramePoint( const std::shared_ptr< MonoFramePoint > &leftPoint, const std::shared_ptr< MonoFramePoint > &rightPoint );

private:
};

}
