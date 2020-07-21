namespace slam {

#include <memory>

class MonoPoint;
class FlowPoint;
class FeaturePoint;
class FramePoint;

class MapPoint;

class FrameBase;
class MonoFrame;
class FlowFrame;
class FeatureFrame;
class ProcessedFrame;
class MonoKeyFrame;
class ProcessedKeyFrame;
class FlowKeyFrame;
class FeatureKeyFrame;
class StereoFrame;
class StereoKeyFrame;
class ProcessedStereoKeyFrame;
class FlowStereoFrame;
class FeatureStereoFrame;
class FlowStereoKeyFrame;
class FeatureStereoKeyFrame;
class ProcessedDenseFrame;
class FlowDenseFrame;
class FeatureDenseFrame;
class FinishedFrame;

class Map;
class FlowMap;
class FeatureMap;

class World;

using MonoPointPtr = std::shared_ptr< MonoPoint >;
using FlowPointPtr = std::shared_ptr< FlowPoint >;
using FeaturePointPtr = std::shared_ptr< FeaturePoint >;
using FramePointPtr = std::shared_ptr< FramePoint >;

using MapPointPtr = std::shared_ptr< MapPoint >;

using FrameBasePtr = std::shared_ptr< FrameBase >;
using MonoFramePtr = std::shared_ptr< MonoFrame >;
using FlowFramePtr = std::shared_ptr< FlowFrame >;
using FeatureFramePtr = std::shared_ptr< FeatureFrame >;
using ProcessedFramePtr = std::shared_ptr< ProcessedFrame >;
using MonoKeyFramePtr = std::shared_ptr< MonoKeyFrame >;
using ProcessedKeyFramePtr = std::shared_ptr< ProcessedKeyFrame >;
using FlowKeyFramePtr = std::shared_ptr< FlowKeyFrame >;
using FeatureKeyFramePtr = std::shared_ptr< FeatureKeyFrame >;
using StereoFramePtr = std::shared_ptr< StereoFrame >;
using StereoKeyFramePtr = std::shared_ptr< StereoKeyFrame >;
using ProcessedStereoKeyFramePtr = std::shared_ptr< ProcessedStereoKeyFrame >;
using FlowStereoFramePtr = std::shared_ptr< FlowStereoFrame >;
using FeatureStereoFramePtr = std::shared_ptr< FeatureStereoFrame >;
using FlowStereoKeyFramePtr = std::shared_ptr< FlowStereoKeyFrame >;
using FeatureStereoKeyFramePtr = std::shared_ptr< FeatureStereoKeyFrame >;
using ProcessedDenseFramePtr = std::shared_ptr< ProcessedDenseFrame >;
using FlowDenseFramePtr = std::shared_ptr< FlowDenseFrame >;
using FeatureDenseFramePtr = std::shared_ptr< FeatureDenseFrame >;
using FinishedFramePtr = std::shared_ptr< FinishedFrame >;

using MapPtr = std::shared_ptr< Map >;
using FlowMapPtr = std::shared_ptr< FlowMap >;
using FeatureMapPtr = std::shared_ptr< FeatureMap >;

using WorldPtr = std::shared_ptr< World >;

}
