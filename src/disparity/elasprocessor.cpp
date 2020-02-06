#include "src/common/precompiled.h"

#include "elasprocessor.h"

#include "src/common/functions.h"

#include "src/libelas/StereoEfficientLargeScale.h"

// ElasDisparityProcessor
ElasDisparityProcessor::ElasDisparityProcessor()
    : DisparityProcessorBase()
{
    initialize();
}

void ElasDisparityProcessor::initialize()
{
    m_matcher = cv::Ptr< StereoEfficientLargeScale >( new StereoEfficientLargeScale() );
}

cv::Mat ElasDisparityProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    cv::Mat dest;

    m_matcher->operator()( left, right, dest, 200 );

    return dest;

}

