#include "src/common/precompiled.h"

#include "framepoint.h"

#include "frame.h"

namespace slam {

    // FramePointBase
    FramePointBase::FramePointBase()
    {
    }

    // MonoFramePoint
    MonoFramePoint::MonoFramePoint( const MonoFrame *parentFrame , const size_t keyPointIndex )
        : m_parentFrame( parentFrame ), m_keyPointIndex( keyPointIndex ), m_parentWorldPoint( nullptr )
    {
    }

    const cv::Point2f &MonoFramePoint::point() const
    {
        return m_parentFrame->m_keyPoints[ m_keyPointIndex ].pt;
    }

    // DoubleFramePointBase
    DoubleFramePointBase::DoubleFramePointBase( const std::shared_ptr< MonoFramePoint > &point1, const std::shared_ptr< MonoFramePoint > &point2 )
        : m_framePoint1( point1 ), m_framePoint2( point2 )
    {
    }

    // StereoFramePoint
    StereoFramePoint::StereoFramePoint(const std::shared_ptr<MonoFramePoint> &leftPoint, const std::shared_ptr<MonoFramePoint> &rightPoint )
        : DoubleFramePointBase( leftPoint, rightPoint )
    {
    }

    const cv::Point2f &StereoFramePoint::leftPoint() const
    {
        return leftFramePoint()->point();
    }

    const cv::Point2f &StereoFramePoint::rightPoint() const
    {
        return rightFramePoint()->point();
    }

    std::shared_ptr< MonoFramePoint > &StereoFramePoint::leftFramePoint()
    {
        return m_framePoint1;
    }

    std::shared_ptr< MonoFramePoint > &StereoFramePoint::rightFramePoint()
    {
        return m_framePoint2;
    }

    const std::shared_ptr< MonoFramePoint > &StereoFramePoint::leftFramePoint() const
    {
        return m_framePoint1;
    }

    const std::shared_ptr< MonoFramePoint > &StereoFramePoint::rightFramePoint() const
    {
        return m_framePoint2;
    }

    // ConsecutiveFramePoint
    ConsecutiveFramePoint::ConsecutiveFramePoint( const std::shared_ptr< MonoFramePoint > &leftPoint, const std::shared_ptr< MonoFramePoint > &rightPoint )
        : DoubleFramePointBase( leftPoint, rightPoint )
    {
    }

}
