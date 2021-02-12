#include <memory>

namespace slam2 {

class System;

class Map;

class Frame;

class FinalFrame;
class ProcFrame;

class StereoFrame;
class FinalStereoFrame;
class ProcStereoFrame;
class ConsecutiveFrame;

class ConsecutiveStereoFrame;

class Track;

class Point2;

class FinalPoint;

class ProcPoint;
class FlowPoint;
class FeaturePoint;

class StereoPoint;
class FinalStereoPoint;
class ProcStereoPoint;
class FlowStereoPoint;
class FeatureStereoPoint;

class MapPoint;

using SystemPtr = std::shared_ptr< System >;

using MapPtr = std::shared_ptr< Map >;

using FramePtr = std::shared_ptr< Frame >;
using FrameConstPtr = std::shared_ptr< const Frame >;

using FinalFramePtr = std::shared_ptr< FinalFrame >;
using FinalFrameConstPtr = std::shared_ptr< const FinalFrame >;
using ProcFramePtr = std::shared_ptr< ProcFrame >;
using ProcFrameConstPtr = std::shared_ptr< const ProcFrame >;

using StereoFramePtr = std::shared_ptr< StereoFrame >;
using FinalStereoFramePtr = std::shared_ptr< FinalStereoFrame >;
using ProcStereoFramePtr = std::shared_ptr< ProcStereoFrame >;
using ProcStereoFrameWeak = std::weak_ptr< ProcStereoFrame >;

using TrackPtr = std::shared_ptr< Track >;
using TrackWeak = std::weak_ptr< Track >;

using Point2Ptr = std::shared_ptr< Point2 >;
using Point2ConstPtr = std::shared_ptr< const Point2 >;
using Point2Weak = std::weak_ptr< Point2 >;

using FinalPointPtr = std::shared_ptr< FinalPoint >;
using FinalPointConstPtr = std::shared_ptr< const FinalPoint >;

using ProcPointPtr = std::shared_ptr< ProcPoint >;
using ProcPointConstPtr = std::shared_ptr< const ProcPoint >;
using FlowPointPtr = std::shared_ptr< FlowPoint >;
using FlowPointConstPtr = std::shared_ptr< const FlowPoint >;
using FeaturePointPtr = std::shared_ptr< FeaturePoint >;
using FeaturePointConstPtr = std::shared_ptr< const FeaturePoint >;

using StereoPointPtr = std::shared_ptr< StereoPoint >;
using StereoPointWeak = std::weak_ptr< StereoPoint >;

using FinalStereoPointPtr = std::shared_ptr< FinalStereoPoint >;
using ProcStereoPointPtr = std::shared_ptr< ProcStereoPoint >;
using FlowStereoPointPtr = std::shared_ptr< FlowStereoPoint >;
using FeatureStereoPointPtr = std::shared_ptr< FeatureStereoPoint >;

using MapPointPtr = std::shared_ptr< MapPoint >;
using MapPointWeak = std::weak_ptr< MapPoint >;

}
