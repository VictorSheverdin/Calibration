#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

class FeatureFrame;
class WorldPoint;
class PointTrack;

class PointBase
{
protected:
    PointBase();
};

class MonoPoint : public PointBase
{
public:
    using TrackPtr = std::weak_ptr< PointTrack >;

    virtual const cv::Point2f &point() const = 0;

    void setTrack( const TrackPtr track );
    TrackPtr track() const;

protected:
    MonoPoint();

    TrackPtr m_track;

};

class FeaturePoint : public MonoPoint
{
public:
    using FramePtr = std::weak_ptr< FeatureFrame >;
    using PointPtr = std::shared_ptr< FeaturePoint >;

    virtual const cv::Point2f &point() const override;

    static PointPtr create( const FramePtr parentFrame, const size_t keyPointIndex );

protected:
    FeaturePoint( const FramePtr parentFrame, const size_t keyPointIndex );

    const FramePtr m_parentFrame; // Parent frame
    size_t m_keyPointIndex; // Index of keypoint in parent frame

private:

};

class Point : public MonoPoint
{
};

class DoublePoint : public PointBase
{
public:
    using MonoPointPtr = std::shared_ptr< MonoPoint >;

    void setMonoPoints( const MonoPointPtr point1, const MonoPointPtr point2 );

    MonoPointPtr monoPoint1() const;
    MonoPointPtr monoPoint2() const;

protected:
    DoublePoint( const MonoPointPtr point1, const MonoPointPtr point2 );

    MonoPointPtr m_point1;
    MonoPointPtr m_point2;

};

class StereoPoint : public DoublePoint
{
    friend class StereoFrame;

public:
    using WorldPointPtr = std::weak_ptr< WorldPoint >;

    const cv::Point2f &leftPoint() const;
    const cv::Point2f &rightPoint() const;

    MonoPointPtr leftMonoPoint() const;
    MonoPointPtr rightMonoPoint() const;

    void setWorldPoint( const WorldPointPtr value );
    WorldPointPtr worldPoint() const;

protected:
    StereoPoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint );

    WorldPointPtr m_worldPoint;

};

class ConsecutivePoint : public DoublePoint
{
    friend class ConsecutiveFrame;

public:

protected:
    ConsecutivePoint( const MonoPointPtr point1, const MonoPointPtr point2 );

};

}
