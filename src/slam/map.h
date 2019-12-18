#pragma once

#include <list>
#include <memory>

#include "src/common/calibrationdatabase.h"

namespace slam {

class DoubleFrame;
class MapPoint;
class World;

class Map : public std::enable_shared_from_this< Map >
{
public:
    using MapPtr = std::shared_ptr< Map >;

    using FramePtr = std::shared_ptr< DoubleFrame >;
    using MapPointPtr = std::shared_ptr< MapPoint >;

    using WorldPtr = std::shared_ptr< World >;

    static MapPtr create( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );

    MapPointPtr createMapPoint( const cv::Point3d &pt, const cv::Scalar &color );

    void removeMapPoint( const MapPointPtr &point );

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::set< MapPointPtr > &mapPoints();
    const std::set< MapPointPtr > &mapPoints() const;

    const FramePtr &backFrame() const;

    void addMapPoint( const MapPointPtr &point );

    const ProjectionMatrix &leftProjectionMatrix() const;
    const ProjectionMatrix &rightProjectionMatrix() const;

    const cv::Mat &baselineVector() const;
    double baselineLenght() const;

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2f &value );

    bool valid() const;

protected:
    using WorldPtrImpl = std::weak_ptr< World >;

    Map( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );

    std::list< FramePtr > m_frames;
    std::set< MapPointPtr > m_mapPoints;

    ProjectionMatrix m_leftProjectionMatrix;
    ProjectionMatrix m_rightProjectionMatrix;

    cv::Mat m_baselineVector;

    size_t m_previousKeypointsCount;

    static const int m_minTrackPoints = 1 << 6;

    static const int m_goodTrackPoints = 1 << 7;
    static const int m_overTrackPoints = 1 << 8;

    static const int m_trackFramePointsCount = 1 << 7;

    static const double m_minTrackInliersRatio;

private:
    void initialize();

};

}
