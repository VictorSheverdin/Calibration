#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "framepoint.h"
#include "worldpoint.h"
#include "src/common/featureprocessor.h"

namespace slam {

class FrameBase
{
public:
    FrameBase();
    virtual ~FrameBase() = default;

protected:
    std::vector< WorldPoint > m_worlPoints;

    cv::Mat m_projectionMatrix;

    cv::Mat m_rotation;
    cv::Mat m_translation;

    static FeatureProcessor m_processor;

private:
};

class MonoFrame : public FrameBase
{
    friend class StereoFrame;
    friend class MonoFramePoint;

public:
    MonoFrame();
    MonoFrame( const CvImage &image );

    void load( const CvImage &image );
    bool drawKeyPoints( CvImage *target ) const;
    bool drawFramePoints( CvImage *target ) const;

protected:
    std::vector< cv::KeyPoint > m_keyPoints;
    cv::Mat m_descriptors;

    std::vector< std::shared_ptr< MonoFramePoint > > m_framePoints;

    std::shared_ptr< MonoFramePoint > createFramePoint( const size_t keyPointIndex );

private:
};

class StereoFrame : public FrameBase
{
    friend class StereoFramePoint;

public:
    StereoFrame();
    StereoFrame( const CvImage &leftImage,  const CvImage &rightImage );

    void load( const CvImage &leftImage,  const CvImage &rightImage );
    CvImage drawKeyPoints( const CvImage &leftImage, const CvImage &rightImage ) const;
    CvImage drawStereoPoints( const CvImage &leftImage, const CvImage &rightImage ) const;

    void triangulatePoints();

protected:
    MonoFrame m_leftFrame;
    MonoFrame m_rightFrame;

    std::vector< std::shared_ptr< StereoFramePoint > > m_framePoints;

    std::shared_ptr< StereoFramePoint > createFramePoint( const std::shared_ptr< MonoFramePoint > &leftPoint, const std::shared_ptr< MonoFramePoint > &rightPoint );

private:
};

}
