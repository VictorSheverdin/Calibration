#pragma once

#include "src/common/image.h"

namespace slam {

class FeatureFrame;
class WorldPoint;

class PointBase
{
protected:
    PointBase();
};

class MonoPoint : public PointBase
{
public:
    using AdjacentPtr = std::shared_ptr< MonoPoint >;
    using WorldPointPtr = std::shared_ptr< WorldPoint >;

    virtual const cv::Point2f &point() const = 0;

    void setStereoPoint( const AdjacentPtr point );
    AdjacentPtr stereoPoint() const;

    void setNextPoint( const AdjacentPtr point );
    AdjacentPtr nextPoint() const;

    void setPrevPoint( const AdjacentPtr point );
    AdjacentPtr prevPoint() const;

    void setWorldPoint( const WorldPointPtr point );
    WorldPointPtr worldPoint() const;

    void setColor( const cv::Scalar &value );
    const cv::Scalar &color() const;

     void drawTrack( CvImage *target ) const;

protected:
    using AdjacentPtrImpl = std::weak_ptr< MonoPoint >;
    using WorldPointPtrImpl = std::weak_ptr< WorldPoint >;

    MonoPoint();
    MonoPoint( const cv::Scalar &color );

    AdjacentPtrImpl m_stereoPoint;
    AdjacentPtrImpl m_nextPoint;
    AdjacentPtrImpl m_prevPoint;

    WorldPointPtrImpl m_worldPoint;

    cv::Scalar m_color;

};

class FeaturePoint : public MonoPoint
{
public:
    using FramePtr = std::weak_ptr< FeatureFrame >;
    using PointPtr = std::shared_ptr< FeaturePoint >;

    virtual const cv::Point2f &point() const override;

    static PointPtr create( const FramePtr parentFrame, const size_t keyPointIndex, const cv::Scalar &color );

protected:
    FeaturePoint( const FramePtr parentFrame, const size_t keyPointIndex, const cv::Scalar &color );

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

    MonoPointPtr monoFramePoint1() const;
    MonoPointPtr monoFramePoint2() const;

protected:
    DoublePoint( const MonoPointPtr point1, const MonoPointPtr point2 );

    MonoPointPtr m_point1;
    MonoPointPtr m_point2;


};

class StereoPoint : public DoublePoint
{
public:
    StereoPoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint );

    MonoPointPtr leftFramePoint() const;
    MonoPointPtr rightFramePoint() const;

    cv::Point2f leftPoint() const;
    cv::Point2f rightPoint() const;

};

class AdjacentPoint : public DoublePoint
{
public:
    AdjacentPoint( const MonoPointPtr previousPoint, const MonoPointPtr nextPoint );

    MonoPointPtr previousFramePoint() const;
    MonoPointPtr nextFramePoint() const;

    cv::Point2f previousPoint() const;
    cv::Point2f nextPoint() const;

};

}
