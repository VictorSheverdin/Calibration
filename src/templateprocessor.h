#pragma once

#include <opencv2/opencv.hpp>

#include "image.h"

class ProcessorData
{
public:
    ProcessorData();

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

    Type type() const;
    const cv::Size &count() const;
    double size() const;
    bool resizeFlag() const;
    unsigned int frameMaximumFlag() const;

    void setAdaptiveThreshold( const bool value );
    void setNormalizeImage( const bool value );
    void setFilterQuads( const bool value );
    void setFastCheck( const bool value );

    bool process(const CvImage &frame, CvImage *procFrame, std::vector<cv::Point2f> *points );

protected:
    Type m_templateType;
    cv::Size m_count;
    double m_size;
    bool m_resizeFlag;
    unsigned int m_frameMaximumSize;
    int m_flags;

private:
    void initialize();
};
