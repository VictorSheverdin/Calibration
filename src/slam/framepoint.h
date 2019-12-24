#pragma once

#include "src/common/image.h"

#include <Eigen/Core>

namespace slam {

class Frame;
class MonoFrame;
class ProcessedFrame;
class MapPoint;

class PointBase
{
protected:
    PointBase();
};

class MonoPoint : public PointBase, public std::enable_shared_from_this< MonoPoint >
{
    friend class MonoFrame;

public:
    using FramePtr = std::shared_ptr< MonoFrame >;

    using AdjacentPtr = std::shared_ptr< MonoPoint >;
    using MapPointPtr = std::shared_ptr< MapPoint >;

    virtual const cv::Point2f &point() const = 0;
    virtual const cv::Scalar &color() const = 0;

    FramePtr parentFrame() const;

    void setStereoPoint( const AdjacentPtr &point );
    void clearStereoPoint();
    AdjacentPtr stereoPoint() const;

    void setNextPoint( const AdjacentPtr &point );
    void clearNextPoint();
    AdjacentPtr nextPoint() const;

    void setPrevPoint( const AdjacentPtr &point );
    void clearPrevPoint();
    AdjacentPtr prevPoint() const;

    void setMapPoint( const MapPointPtr &point );
    void clearMapPoint();
    MapPointPtr mapPoint() const;

    size_t connectedPointsCount() const;

    void drawTrack( CvImage *target , const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) ) const;

    Eigen::Matrix< double, 2, 1 > eigenPoint() const;
    Eigen::Matrix< double, 3, 1 > eigenStereoPoint() const;

    double bf() const;

protected:
    using FramePtrImpl = std::weak_ptr< MonoFrame >;

    using AdjacentPtrImpl = std::weak_ptr< MonoPoint >;
    using MapPointPtrImpl = std::weak_ptr< MapPoint >;

    MonoPoint( const FramePtr &parentFrame );

    const FramePtrImpl m_parentFrame; // Parent frame

    AdjacentPtrImpl m_stereoPoint;
    AdjacentPtrImpl m_nextPoint;
    AdjacentPtrImpl m_prevPoint;

    MapPointPtrImpl m_mapPoint;

    void drawPrevTrack( CvImage *target , const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) ) const;
    void drawNextTrack( CvImage *target , const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) ) const;

    void initialize();

};

class ProcessedPoint : public MonoPoint
{
public:
    using FramePtr = std::shared_ptr< ProcessedFrame >;
    using PointPtr = std::shared_ptr< ProcessedPoint >;

    virtual const cv::Point2f &point() const override;
    virtual const cv::Scalar &color() const override;

    const cv::KeyPoint &keyPoint() const;

    static PointPtr create( const FramePtr &parentFrame, const size_t keyPointIndex );

    FramePtr parentFrame() const;

    bool lastTriangulated() const;

protected:
    using FramePtrImpl = std::weak_ptr< ProcessedFrame >;

    ProcessedPoint( const FramePtr &parentFrame, const size_t keyPointIndex );

    size_t m_keyPointIndex; // Index of keypoint in parent frame

};

class FramePoint : public MonoPoint
{
public:
    using FramePtr = std::shared_ptr< Frame >;
    using PointPtr = std::shared_ptr< FramePoint >;

    using ProcessedPointPtr = std::shared_ptr< ProcessedPoint >;

    virtual const cv::Point2f &point() const override;
    virtual const cv::Scalar &color() const override;

    static PointPtr create( const FramePtr &parentFrame );
    static PointPtr create( const FramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color );

    FramePtr parentFrame() const;

    void replace( const ProcessedPointPtr &point );

    void setPoint( const cv::Point2f &point );
    void setColor( const cv::Scalar &color );

    void set( const cv::Point2f &point, const cv::Scalar &color );

protected:
    FramePoint( const FramePtr &parentFrame );
    FramePoint( const FramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color );

    cv::Point2f m_point;
    cv::Scalar m_color;

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
