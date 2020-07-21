#pragma once

#include "src/common/image.h"
#include "src/common/colorpoint.h"

#include <Eigen/Core>

#include "alias.h"

namespace slam {

class MonoFrame;
class FlowFrame;
class FeatureFrame;
class FinishedFrame;
class MapPoint;

// Базовый класс для реализации точек на изображении
class PointBase
{
public:
    using ObjectPtr = std::shared_ptr< PointBase >;
    using ObjectConstPtr = std::shared_ptr< const PointBase >;

protected:
    PointBase() = default;
};

class MonoPoint : public PointBase, public std::enable_shared_from_this< MonoPoint >
{
    friend class MonoFrame;

public:
    using ObjectPtr = std::shared_ptr< MonoPoint >;
    using ObjectConstPtr = std::shared_ptr< const MonoPoint >;

    virtual const cv::Point2f &point() const = 0;
    virtual const cv::Scalar &color() const = 0;

    void setParentFrame( const MonoFramePtr &parent );
    MonoFramePtr parentFrame() const;

    void setStereoPoint( const MonoPointPtr &point );
    void clearStereoPoint();
    MonoPointPtr stereoPoint() const;

    void setNextPoint( const MonoPointPtr &point );
    void clearNextPoint();
    MonoPointPtr nextPoint() const;

    void setPrevPoint( const MonoPointPtr &point );
    void clearPrevPoint();
    MonoPointPtr prevPoint() const;

    void setMapPoint( const MapPointPtr &point );
    void clearMapPoint();
    MapPointPtr mapPoint() const;

    size_t connectedPointsCount() const;

    size_t prevTrackLenght() const;
    size_t nextTrackLenght() const;

    void drawTrack( CvImage *target , const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) ) const;

    Eigen::Matrix< double, 2, 1 > eigenPoint() const;
    Eigen::Matrix< double, 3, 1 > eigenStereoPoint() const;

    double error() const;
    void setError( const double value );

    void dissolve();

protected:
    using FramePtrImpl = std::weak_ptr< MonoFrame >;

    using AdjacentPtrImpl = std::weak_ptr< MonoPoint >;
    using MapPointPtrImpl = std::weak_ptr< MapPoint >;

    MonoPoint( const MonoFramePtr &parentFrame );

    FramePtrImpl m_parentFrame; // Parent frame

    AdjacentPtrImpl m_stereoPoint;
    AdjacentPtrImpl m_nextPoint;
    AdjacentPtrImpl m_prevPoint;

    MapPointPtrImpl m_mapPoint;

    double m_error;

    void drawPrevTrack( CvImage *target , const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) ) const;
    void drawNextTrack( CvImage *target , const cv::Scalar &color = cv::Scalar( 0, 255, 0, 100 ) ) const;

    void initialize();

};

class ProcessedPointBase : public MonoPoint
{
public:
    using ObjectPtr = std::shared_ptr< ProcessedPointBase >;
    using ObjectConstPtr = std::shared_ptr< const ProcessedPointBase >;

protected:
    ProcessedPointBase( const MonoFramePtr &parentFrame );

};

class FlowPoint : public ProcessedPointBase
{
public:
    using ObjectPtr = std::shared_ptr< FlowPoint >;
    using ObjectConstPtr = std::shared_ptr< const FlowPoint >;

    virtual const cv::Point2f &point() const override;
    virtual const cv::Scalar &color() const override;

    static ObjectPtr create( const FlowFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color );

    FlowFramePtr parentFrame() const;

protected:
    FlowPoint( const FlowFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color );

    cv::Point2f m_point;
    cv::Scalar m_color;

};

class FeaturePoint : public ProcessedPointBase
{
public:
    using ObjectPtr = std::shared_ptr< FeaturePoint >;
    using ObjectConstPtr = std::shared_ptr< const FeaturePoint >;

    virtual const cv::Point2f &point() const override;
    virtual const cv::Scalar &color() const override;

    const cv::KeyPoint &keyPoint() const;
    cv::Mat descriptor() const;

    static ObjectPtr create( const FeatureFramePtr &parentFrame, const size_t keyPointIndex );

    FeatureFramePtr parentFrame() const;

protected:
    using FramePtrImpl = std::weak_ptr< FeatureFrame >;

    FeaturePoint( const FeatureFramePtr &parentFrame, const size_t keyPointIndex );

    size_t m_keyPointIndex; // Index of keypoint in parent frame

};

class FramePoint : public MonoPoint, public ColorPoint2d
{
public:
    using ObjectPtr = std::shared_ptr< FramePoint >;
    using ObjectConstPtr = std::shared_ptr< const FramePoint >;

    virtual const cv::Point2f &point() const override;
    virtual const cv::Scalar &color() const override;

    static ObjectPtr create( const FinishedFramePtr &parentFrame );
    static ObjectPtr create( const FinishedFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color );

    FinishedFramePtr parentFrame() const;

    void replace( const MonoPointPtr &point );

protected:
    FramePoint( const FinishedFramePtr &parentFrame );
    FramePoint( const FinishedFramePtr &parentFrame, const cv::Point2f &point, const cv::Scalar &color );

};

class DoublePoint : public PointBase
{
public:
    using ObjectPtr = std::shared_ptr< DoublePoint >;
    using ObjectConstPtr = std::shared_ptr< const DoublePoint >;

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
    using ObjectPtr = std::shared_ptr< StereoPoint >;
    using ObjectConstPtr = std::shared_ptr< const StereoPoint >;

    StereoPoint( const MonoPointPtr leftPoint, const MonoPointPtr rightPoint );

    MonoPointPtr leftFramePoint() const;
    MonoPointPtr rightFramePoint() const;

    cv::Point2f leftPoint() const;
    cv::Point2f rightPoint() const;

};

class ConsecutivePoint : public DoublePoint
{
public:
    using ObjectPtr = std::shared_ptr< ConsecutivePoint >;
    using ObjectConstPtr = std::shared_ptr< const ConsecutivePoint >;

    ConsecutivePoint( const MonoPointPtr previousPoint, const MonoPointPtr nextPoint );

    MonoPointPtr startFramePoint() const;
    MonoPointPtr endFramePoint() const;

    cv::Point2f startPoint() const;
    cv::Point2f endPoint() const;

};

}
