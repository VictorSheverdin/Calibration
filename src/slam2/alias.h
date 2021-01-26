#include <memory>

namespace slam2 {

class System;

class Map;

class Frame;

class FinalFrame;
class ProcFrame;

class StereoFrame;
class ProcStereoFrame;
class FinalStereoFrame;

class Track;

class Point2;

class FramePoint;

class ProcPoint;
class FlowPoint;
class FeaturePoint;

class MapPoint;

using SystemPtr = std::shared_ptr< System >;

using MapPtr = std::shared_ptr< Map >;

using FramePtr = std::shared_ptr< Frame >;

using FinalFramePtr = std::shared_ptr< FinalFrame >;
using ProcFramePtr = std::shared_ptr< ProcFrame >;

using StereoFramePtr = std::shared_ptr< StereoFrame >;
using FinalStereoFramePtr = std::shared_ptr< FinalStereoFrame >;
using ProcStereoFramePtr = std::shared_ptr< ProcStereoFrame >;

using TrackPtr = std::shared_ptr< Track >;

using Point2Ptr = std::shared_ptr< Point2 >;
using Point2Weak = std::weak_ptr< Point2 >;

using FramePointPtr = std::shared_ptr< FramePoint >;

using ProcPointPtr = std::shared_ptr< ProcPoint >;
using FlowPointPtr = std::shared_ptr< FlowPoint >;
using FeaturePointPtr = std::shared_ptr< FeaturePoint >;

using MapPointPtr = std::shared_ptr< MapPoint >;

}
