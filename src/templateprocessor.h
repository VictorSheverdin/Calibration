#pragma once

#include <opencv2/opencv.hpp>

#include "image.h"

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
    enum Type { CHECKERBOARD, CIRCLES, ASYM_CIRCLES };

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

    unsigned int m_frameMaximumSize;
    int m_flags;

    bool findPoints( const CvImage &frame, std::vector<cv::Point2f> *points );

private:
    void initialize();
};
