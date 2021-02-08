#pragma once

#include "src/common/featureprocessor.h"

#include "alias.h"

namespace slam2 {

class Tracker
{
public:
    virtual ~Tracker() = default;

    virtual void extract( ProcStereoFrame *frame ) = 0;
    virtual void extract( ConsecutiveFrame *frame ) = 0;

    virtual void match( ProcStereoFrame *frame ) = 0;
    virtual void match( ConsecutiveFrame *frame ) = 0;

    virtual void track( ConsecutiveStereoFrame &frames );

protected:
    Tracker() = default;

};

class FlowTracker : public Tracker
{
public:
    double extractPrecision() const;
    void setExtractPrecision( const double value );

    void setExtractionDistance( const double value );
    double extractionDistance() const;

    size_t winSize() const;
    void setWinSize( const size_t value );

    size_t levels() const;
    void setLevels( const size_t value );

    void setRansacReprojectionThreshold( const double &value );
    double ransacReprojectionThreshold() const;

    void setRansacConfidence( const double &value );
    double ransacConfidence() const;

    void track( ConsecutiveStereoFrame &frames ) override;

protected:
    FlowTracker();

    std::unique_ptr< FastProcessor > _pointsDetector;
    std::unique_ptr< FlowProcessor > _flowProcessor;

    void extractPoints( ProcFrame *frame );

private:
    void initialize();
};

class CPUFlowTracker : public FlowTracker
{
public:
    CPUFlowTracker();

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrame *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrame *frame ) override;

protected:
    CPUFlowProcessor *processor() const;

    void prepareFrame( ProcFrame *frame );

private:
    void initialize();

};

class GPUFlowTracker : public FlowTracker
{
public:
    GPUFlowTracker();

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrame *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrame *frame ) override;

protected:
    GPUFlowProcessor *processor() const;

private:
    void initialize();

};

class FeatureTracker : public Tracker
{
public:
    void setRansacReprojectionThreshold( const double &value );
    double ransacReprojectionThreshold() const;

    void setRansacConfidence( const double &value );
    double ransacConfidence() const;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrame *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrame *frame ) override;

    void track( ConsecutiveStereoFrame &frames ) override;

protected:
    FeatureTracker() = default;

    std::unique_ptr< FullProcessor > _descriptorProcessor;
    std::unique_ptr< DescriptorMatcher > _featuresMatcher;

    void extract( ProcFrame *frame );

};

class SiftTracker : public FeatureTracker
{
public:
    SiftTracker();

protected:
    SiftProcessor *processor() const;
    FlannMatcher *matcher() const;

private:
    void initialize();

};

class OrbTracker : public FeatureTracker
{
public:
    OrbTracker();

protected:
    OrbProcessor *processor() const;
    BFMatcher *matcher() const;

private:
    void initialize();

};

class AKazeTracker : public FeatureTracker
{
public:
    AKazeTracker();

protected:
    AKazeProcessor *processor() const;
    BFMatcher *matcher() const;

private:
    void initialize();

};

class SuperGlueTracker : public Tracker
{
public:
    SuperGlueTracker();

    void setRansacReprojectionThreshold( const double &value );
    double ransacReprojectionThreshold() const;

    void setRansacConfidence( const double &value );
    double ransacConfidence() const;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrame *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrame *frame ) override;

    void track( ConsecutiveStereoFrame &frames ) override;

protected:
    SuperGlueProcessor *processor() const;

    std::unique_ptr< SuperGlueProcessor > _processor;

    void extract( ProcFrame *frame );

private:
    void initialize();

};

}
