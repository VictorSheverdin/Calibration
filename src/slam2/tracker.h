#pragma once

#include "src/common/featureprocessor.h"

#include "alias.h"

namespace slam2 {

class Tracker
{
public:
    virtual ~Tracker() = default;

    virtual void prepareFrame( ProcStereoFrame *frame ) = 0;
    virtual void extractFeatures( ProcStereoFrame *frame ) = 0;
    virtual void match( ProcStereoFrame *frame ) = 0;
    virtual void match( ConsecutiveStereoFrames *frame ) = 0;

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
    void extractFeatures( ProcStereoFrame *frame ) override;
    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveStereoFrames *frame ) override;

protected:
    CPUFlowProcessor *processor() const;

private:
    void initialize();

};

class FeatureTracker : public Tracker
{
public:

protected:
    FeatureTracker() = default;

    std::unique_ptr< FullProcessor > _descriptorProcessor;

};

class SiftTracker : public FeatureTracker
{
public:
    SiftTracker();

    void prepareFrame( ProcStereoFrame *frame ) override;
    void extractFeatures( ProcStereoFrame *frame ) override;
    void match( ProcStereoFrame *frame ) override;
    void match( ConsecutiveStereoFrames *frame ) override;

protected:
    SiftProcessor *processor() const;
    FlannMatcher *matcher() const;

    std::unique_ptr< DescriptorMatcher > _featuresMatcher;

private:
    void initialize();

};

}
