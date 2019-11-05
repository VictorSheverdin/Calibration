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
    friend class ConsecutiveFrame;
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

class DoubleFrameBase : public FrameBase
{
public:
    DoubleFrameBase();
    DoubleFrameBase( const CvImage &image1, const CvImage &image2 );

    void load( const CvImage &image1, const CvImage &image2 );

protected:
    std::shared_ptr< MonoFrame > m_frame1;
    std::shared_ptr< MonoFrame > m_frame2;

};

class StereoFrame : public DoubleFrameBase
{
    friend class StereoFramePoint;

public:
    StereoFrame();
    StereoFrame( const CvImage &leftImage, const CvImage &rightImage );

    CvImage drawKeyPoints( const CvImage &leftImage, const CvImage &rightImage ) const;
    CvImage drawStereoPoints( const CvImage &leftImage, const CvImage &rightImage ) const;

    void triangulatePoints();

protected:
    void setLeftFrame( const std::shared_ptr< MonoFrame > &value );
    void setRightFrame( const std::shared_ptr< MonoFrame > &value );

    std::shared_ptr< MonoFrame > &leftFrame();
    std::shared_ptr< MonoFrame > &rightFrame();

    const std::shared_ptr< MonoFrame > &leftFrame() const;
    const std::shared_ptr< MonoFrame > &rightFrame() const;

    std::vector< std::shared_ptr< StereoFramePoint > > m_framePoints;

    std::shared_ptr< StereoFramePoint > createFramePoint( const std::shared_ptr< MonoFramePoint > &leftPoint, const std::shared_ptr< MonoFramePoint > &rightPoint );

private:
};

class ConsecutiveFrame : public DoubleFrameBase
{
    friend class ConsecutiveFramePoint;

public:
    ConsecutiveFrame();
    ConsecutiveFrame( const CvImage &leftImage, const CvImage &rightImage );

    CvImage drawTrack( const CvImage &image ) const;

    void recoverPose();

protected:
    std::vector< std::shared_ptr< ConsecutiveFramePoint > > m_framePoints;

    std::shared_ptr< ConsecutiveFramePoint > createFramePoint( const std::shared_ptr< MonoFramePoint > &point1, const std::shared_ptr< MonoFramePoint > &point2 );

};

}
