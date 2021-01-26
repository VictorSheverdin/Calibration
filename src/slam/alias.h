namespace slam {

#include <memory>

class MonoPoint;
class FlowPoint;
class FeaturePoint;
class FinishedFramePoint;

class MapPoint;

class FrameBase;
class MonoFrame;
class FlowFrame;
class ProcessedFrame;
class MonoKeyFrame;
class ProcessedKeyFrame;
class FlowKeyFrame;
class StereoFrame;
class StereoKeyFrame;
class ProcessedStereoFrame;
class ProcessedStereoKeyFrame;
class FlowStereoFrame;
class FlowStereoKeyFrame;
class ProcessedDenseFrame;
class FlowDenseFrame;
class FinishedFrame;
class FinishedKeyFrame;

class World;
class Map;

using MonoPointPtr = std::shared_ptr< MonoPoint >;
using FlowPointPtr = std::shared_ptr< FlowPoint >;
using FeaturePointPtr = std::shared_ptr< FeaturePoint >;
using FinishedFramePointPtr = std::shared_ptr< FinishedFramePoint >;

using MapPointPtr = std::shared_ptr< MapPoint >;

using FrameBasePtr = std::shared_ptr< FrameBase >;
using MonoFramePtr = std::shared_ptr< MonoFrame >;
using FlowFramePtr = std::shared_ptr< FlowFrame >;
using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;
using MonoKeyFramePtr = std::shared_ptr< MonoKeyFrame >;
using ProcessedKeyFramePtr = std::shared_ptr< ProcessedKeyFrame >;
using FlowKeyFramePtr = std::shared_ptr< FlowKeyFrame >;
using StereoFramePtr = std::shared_ptr< StereoFrame >;
using StereoKeyFramePtr = std::shared_ptr< StereoKeyFrame >;
using ProcessedStereoFramePtr = std::shared_ptr< ProcessedStereoFrame >;
using ProcessedStereoKeyFramePtr = std::shared_ptr< ProcessedStereoKeyFrame >;
using FlowStereoFramePtr = std::shared_ptr< FlowStereoFrame >;
using FlowStereoKeyFramePtr = std::shared_ptr< FlowStereoKeyFrame >;
using ProcessedDenseFramePtr = std::shared_ptr< ProcessedDenseFrame >;
using FlowDenseFramePtr = std::shared_ptr< FlowDenseFrame >;
using FinishedFramePtr = std::shared_ptr< FinishedFrame >;
using FinishedKeyFramePtr = std::shared_ptr< FinishedKeyFrame >;

using WorldPtr = std::shared_ptr< World >;

using MapPtr = std::shared_ptr< Map >;

}
