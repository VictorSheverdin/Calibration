#pragma once

#include "src/common/featureprocessor.h"

#include <QThread>
#include <QMutex>

struct ProcessorResult
{
public:
    unsigned int count;
    double procTime;
    CvImage result;
};

class ProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit ProcessorThread( QObject *parent = nullptr );

    bool process( const CvImage &img1, const CvImage &img2 );

    void setDetector( const std::shared_ptr< FeatureProcessorBase > &processor );
    void setDescriptor( const std::shared_ptr< FeatureProcessorBase > &processor );
    void setMatcher( std::shared_ptr< DescriptorMatcher > &matcher );

    ProcessorResult result();

signals:
    void frameProcessed();

protected:
    CvImage m_img1;
    CvImage m_img2;
    QMutex m_processMutex;

    ProcessorResult m_result;
    QMutex m_resultMutex;

    std::shared_ptr< FeatureProcessorBase > m_detector;
    std::shared_ptr< FeatureProcessorBase > m_descriptor;
    std::shared_ptr< DescriptorMatcher > m_matcher;

    virtual void run() override;

private:
    void initialize();

};
