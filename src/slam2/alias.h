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
class ConsecutiveFrames;

class Track;

class Point2;

class FramePoint;

class ProcPoint;
class FlowPoint;
class FeaturePoint;

class StereoPoint;
class ProcStereoPoint;
class FlowStereoPoint;
class FeatureStereoPoint;

class MapPoint;

using SystemPtr = std::shared_ptr< System >;

using MapPtr = std::shared_ptr< Map >;

using FramePtr = std::shared_ptr< Frame >;

using FinalFramePtr = std::shared_ptr< FinalFrame >;
using ProcFramePtr = std::shared_ptr< ProcFrame >;

using StereoFramePtr = std::shared_ptr< StereoFrame >;
using FinalStereoFramePtr = std::shared_ptr< FinalStereoFrame >;
using ProcStereoFramePtr = std::shared_ptr< ProcStereoFrame >;
using ProcStereoFrameWeak = std::weak_ptr< ProcStereoFrame >;

using TrackPtr = std::shared_ptr< Track >;
using TrackWeak = std::weak_ptr< Track >;

using Point2Ptr = std::shared_ptr< Point2 >;
using Point2Weak = std::weak_ptr< Point2 >;

using FramePointPtr = std::shared_ptr< FramePoint >;

using ProcPointPtr = std::shared_ptr< ProcPoint >;
using FlowPointPtr = std::shared_ptr< FlowPoint >;
using FeaturePointPtr = std::shared_ptr< FeaturePoint >;

using StereoPointPtr = std::shared_ptr< StereoPoint >;
using StereoPointWeak = std::weak_ptr< StereoPoint >;

using ProcStereoPointPtr = std::shared_ptr< ProcStereoPoint >;
using FlowStereoPointPtr = std::shared_ptr< FlowStereoPoint >;
using FeatureStereoPointPtr = std::shared_ptr< FeatureStereoPoint >;

using MapPointPtr = std::shared_ptr< MapPoint >;
using MapPointWeak = std::weak_ptr< MapPoint >;

}
