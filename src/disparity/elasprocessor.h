#pragma once

#include "src/common/stereoprocessor.h"

class StereoEfficientLargeScale;

class ElasDisparityProcessor : public DisparityProcessorBase
{
public:
    ElasDisparityProcessor();

    virtual cv::Mat processDisparity( const CvImage &left, const CvImage &right ) override;

protected:
    cv::Ptr< StereoEfficientLargeScale > m_matcher;

private:
    void initialize();

};
