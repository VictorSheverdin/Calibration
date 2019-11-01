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

    // StereoFramePoint
    StereoFramePoint::StereoFramePoint(const std::shared_ptr<MonoFramePoint> &leftPoint, const std::shared_ptr<MonoFramePoint> &rightPoint )
        : m_leftFramePoint( leftPoint ), m_rightFramePoint( rightPoint )
    {
    }

    const cv::Point2f &StereoFramePoint::leftPoint() const
    {
        return m_leftFramePoint->point();
    }

    const cv::Point2f &StereoFramePoint::rightPoint() const
    {
        return m_rightFramePoint->point();
    }


}
