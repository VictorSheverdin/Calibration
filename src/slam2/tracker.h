#pragma once

#include "src/common/featureprocessor.h"

#include "alias.h"

namespace slam2 {

class Tracker
{
public:
    virtual ~Tracker() = default;

    virtual void prepareFrame( ProcStereoFrame *frame ) = 0;
    virtual void prepareFrame( ConsecutiveFrames *frame ) = 0;

    virtual void extract( ProcStereoFrame *frame ) = 0;
    virtual void extract( ConsecutiveFrames *frame ) = 0;

    virtual void match( ProcStereoFrame *frame ) = 0;
    virtual void match( ConsecutiveFrames *frame ) = 0;

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

protected:
    FlowTracker() = default;

    std::unique_ptr< FlowProcessor > _pointsProcessor;

};

class CPUFlowTracker : public FlowTracker
{
public:
    CPUFlowTracker();

    void prepareFrame( ProcStereoFrame *frame ) override;
    void prepareFrame( ConsecutiveFrames *frame ) override;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrames *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrames *frame ) override;

protected:
    CPUFlowProcessor *processor() const;

    void prepareFrame( ProcFrame *frame );
    void extract( ProcFrame *frame );

private:
    void initialize();

};

class GPUFlowTracker : public FlowTracker
{
public:
    GPUFlowTracker();

    void prepareFrame( ProcStereoFrame *frame ) override;
    void prepareFrame( ConsecutiveFrames *frame ) override;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrames *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrames *frame ) override;

protected:
    GPUFlowProcessor *processor() const;

    void prepareFrame( ProcFrame *frame );
    void extract( ProcFrame *frame );

private:
    void initialize();

};

class FeatureTracker : public Tracker
{
public:

protected:
    FeatureTracker() = default;

    std::unique_ptr< FullProcessor > _descriptorProcessor;
    std::unique_ptr< DescriptorMatcher > _featuresMatcher;

};

class SiftTracker : public FeatureTracker
{
public:
    SiftTracker();

    void prepareFrame( ProcStereoFrame *frame ) override;
    void prepareFrame( ConsecutiveFrames *frame ) override;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrames *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrames *frame ) override;

protected:
    SiftProcessor *processor() const;
    FlannMatcher *matcher() const;

    void prepareFrame( ProcFrame *frame );
    void extract( ProcFrame *frame );

private:
    void initialize();

};

class OrbTracker : public FeatureTracker
{
public:
    OrbTracker();

    void prepareFrame( ProcStereoFrame *frame ) override;
    void prepareFrame( ConsecutiveFrames *frame ) override;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrames *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrames *frame ) override;

protected:
    OrbProcessor *processor() const;
    BFMatcher *matcher() const;

    void prepareFrame( ProcFrame *frame );
    void extract( ProcFrame *frame );

private:
    void initialize();

};

class AKazeTracker : public FeatureTracker
{
public:
    AKazeTracker();

    void prepareFrame( ProcStereoFrame *frame ) override;
    void prepareFrame( ConsecutiveFrames *frame ) override;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrames *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrames *frame ) override;

protected:
    AKazeProcessor *processor() const;
    BFMatcher *matcher() const;

    void prepareFrame( ProcFrame *frame );
    void extract( ProcFrame *frame );

private:
    void initialize();

};

class SuperGlueTracker : public Tracker
{
public:
    SuperGlueTracker();

    void prepareFrame( ProcStereoFrame *frame ) override;
    void prepareFrame( ConsecutiveFrames *frame ) override;

    void extract( ProcStereoFrame *frame ) override;
    void extract( ConsecutiveFrames *frame ) override;

    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveFrames *frame ) override;

protected:
    SuperGlueProcessor *processor() const;

    std::unique_ptr< SuperGlueProcessor > _processor;

    void prepareFrame( ProcFrame *frame );
    void extract( ProcFrame *frame );

private:
    void initialize();

};

}
