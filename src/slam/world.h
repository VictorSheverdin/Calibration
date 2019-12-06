#pragma once

#include "src/common/calibrationdatabase.h"
#include "src/common/rectificationprocessor.h"

#include "map.h"

namespace slam {

class World : public std::enable_shared_from_this< World >
{
public:

    using WorldPtr = std::shared_ptr< World >;

    using FramePtr = Map::FramePtr;

    using MapPtr = Map::MapPtr;
    using MapPointPtr = Map::MapPointPtr;

    static WorldPtr create( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );

    std::list< FramePtr > &frames();
    const std::list< FramePtr > &frames() const;

    std::set< MapPointPtr > &mapPoints();
    const std::set< MapPointPtr > &mapPoints() const;

    bool track( const CvImage &leftImage, const CvImage &rightImage );

    const ProjectionMatrix &leftProjectionMatrix() const;
    const ProjectionMatrix &rightProjectionMatrix() const;

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2f &value );

    void createMap();

    const cv::Mat &baselineVector() const;
    double baselineLenght() const;

protected:
    World( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );

    ProjectionMatrix m_leftProjectionMatrix;
    ProjectionMatrix m_rightProjectionMatrix;

    cv::Mat m_baselineVector;

    MapPtr m_map;

    static const int m_keypointsCount = 25000;

private:
    void initialize();

};

}
