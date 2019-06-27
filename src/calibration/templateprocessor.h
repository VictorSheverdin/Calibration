#pragma once

#include <QThread>
#include <QComboBox>

#include "src/common/image.h"

class ProcessorState
{
public:
    ProcessorState();

protected:

private:
    void initialize();
};

class TemplateProcessor
{
public:
    enum Type { CHECKERBOARD, CIRCLES, ASYM_CIRCLES, ARUCO_MARKERS };

    TemplateProcessor();

    void setType( const Type type );
    void setCount( const cv::Size &count );
    void setSize( const double value );
    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );

    void setAdaptiveThreshold( const bool value );
    void setNormalizeImage( const bool value );
    void setFilterQuads( const bool value );
    void setFastCheck( const bool value );

    Type type() const;
    const cv::Size &count() const;
    double size() const;
    bool resizeFlag() const;
    unsigned int frameMaximumFlag() const;

    bool adaptiveThreshold() const;
    bool normalizeImage() const;
    bool filterQuads() const;
    bool fastCheck() const;

    bool processFrame( const CvImage &frame, CvImage *view, std::vector< cv::Point2f > *points );
    bool processPreview( const CvImage &frame, CvImage *preview, std::vector< cv::Point2f > *points );

    bool calcChessboardCorners(std::vector< cv::Point3f > *corners);

protected:
    Type m_templateType;
    cv::Size m_count;
    double m_size;
    bool m_resizeFlag;

    bool m_subPixFlag;
    cv::Size m_subPixWinSize;
    cv::Size m_subPixZeroZone;

    int m_frameMaximumSize;
    int m_flags;

    bool findPoints( const CvImage &frame, std::vector<cv::Point2f> *points );

private:
    void initialize();
};

class ProcessorThread : public QThread
{
    Q_OBJECT

public:
    explicit ProcessorThread( QObject *parent = nullptr );

protected:
    virtual void run() override;

private:
    void initialize();

};
