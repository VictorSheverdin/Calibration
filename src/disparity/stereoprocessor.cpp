#include "src/common/precompiled.h"

#include "stereoprocessor.h"

// BMProcessor
BMProcessor::BMProcessor()
{
    initialize();
}

void BMProcessor::initialize()
{
    m_matcher = cv::StereoBM::create();
}

int BMProcessor::getMinDisparity() const
{
    return m_matcher->getMinDisparity();
}

void BMProcessor::setMinDisparity( const int minDisparity )
{
    m_matcher->setMinDisparity( minDisparity );
}

int BMProcessor::getNumDisparities() const
{
    return m_matcher->getNumDisparities();
}

void BMProcessor::setNumDisparities( const int numDisparities )
{
    m_matcher->setNumDisparities( numDisparities );
}

int BMProcessor::getBlockSize() const
{
    return m_matcher->getBlockSize();
}

void BMProcessor::setBlockSize( const int blockSize )
{
    m_matcher->setBlockSize( blockSize );
}

int BMProcessor::getSpeckleWindowSize() const
{
    return m_matcher->getSpeckleWindowSize();
}

void BMProcessor::setSpeckleWindowSize( const int speckleWindowSize )
{
    m_matcher->setSpeckleWindowSize( speckleWindowSize );
}

int BMProcessor::getSpeckleRange() const
{
    return m_matcher->getSpeckleRange();
}

void BMProcessor::setSpeckleRange( const int speckleRange )
{
    m_matcher->setSpeckleRange( speckleRange );
}

int BMProcessor::getDisp12MaxDiff() const
{
    return m_matcher->getDisp12MaxDiff();
}

void BMProcessor::setDisp12MaxDiff( const int disp12MaxDiff )
{
    m_matcher->setDisp12MaxDiff( disp12MaxDiff );
}

int BMProcessor::getPreFilterType() const
{
    return m_matcher->getPreFilterType();
}

void BMProcessor::setPreFilterType( const int preFilterType )
{
    m_matcher->setPreFilterType( preFilterType );
}

int BMProcessor::getPreFilterSize() const
{
    return m_matcher->getPreFilterSize();
}

void BMProcessor::setPreFilterSize( const int preFilterSize )
{
    m_matcher->setPreFilterSize( preFilterSize );
}

int BMProcessor::getPreFilterCap() const
{
    return m_matcher->getPreFilterCap();
}

void BMProcessor::setPreFilterCap( const int preFilterCap )
{
    m_matcher->setPreFilterCap( preFilterCap );
}

int BMProcessor::getTextureThreshold() const
{
    return m_matcher->getTextureThreshold();
}

void BMProcessor::setTextureThreshold( const int textureThreshold )
{
    m_matcher->setTextureThreshold( textureThreshold );
}

int BMProcessor::getUniquenessRatio() const
{
    return m_matcher->getUniquenessRatio();
}

void BMProcessor::setUniquenessRatio( const int uniquenessRatio )
{
    m_matcher->setUniquenessRatio( uniquenessRatio );
}

cv::Rect BMProcessor::getROI1() const
{
    return m_matcher->getROI1();
}

void BMProcessor::setROI1( const cv::Rect &roi1 )
{
    m_matcher->setROI1( roi1 );
}

cv::Rect BMProcessor::getROI2() const
{
    return m_matcher->getROI2();
}

void BMProcessor::setROI2( const cv::Rect &roi2 )
{
    m_matcher->setROI2( roi2 );
}

cv::Mat BMProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    CvImage leftGray;
    CvImage rightGray;

    cv::cvtColor( left,  leftGray,  cv::COLOR_BGR2GRAY );
    cv::cvtColor( right, rightGray, cv::COLOR_BGR2GRAY );

    cv::Mat leftDisp;
    cv::Mat rightDisp;

    m_matcher->compute( leftGray, rightGray, leftDisp );

/*    cv::Ptr< cv::ximgproc::DisparityWLSFilter > wlsFilter;
    wlsFilter = cv::ximgproc::createDisparityWLSFilter( m_matcher );

    auto rightMatcher = cv::ximgproc::createRightMatcher(m_matcher);
    rightMatcher->compute( rightGray, leftGray, rightDisp );

    wlsFilter->setLambda( 8000.0 );
    wlsFilter->setSigmaColor( 1.5 );

    cv::Mat filteredDisp;

    wlsFilter->filter( leftDisp, left, filteredDisp, rightDisp );*/

    return leftDisp;

}

// GMProcessor
GMProcessor::GMProcessor()
{
    initialize();
}

void GMProcessor::initialize()
{
    m_matcher = cv::StereoSGBM::create();
}

int GMProcessor::getMode() const
{
    return m_matcher->getMode();
}

void GMProcessor::setMode( int mode )
{
    m_matcher->setMode( mode );
}

int GMProcessor::getMinDisparity() const
{
    return m_matcher->getMinDisparity();
}

void GMProcessor::setMinDisparity( const int minDisparity )
{
    m_matcher->setMinDisparity( minDisparity );
}

int GMProcessor::getNumDisparities() const
{
    return m_matcher->getNumDisparities();
}

void GMProcessor::setNumDisparities( const int numDisparities )
{
    m_matcher->setNumDisparities( numDisparities );
}

int GMProcessor::getBlockSize() const
{
    return m_matcher->getBlockSize();
}

void GMProcessor::setBlockSize( const int blockSize )
{
    m_matcher->setBlockSize( blockSize );
}

int GMProcessor::getSpeckleWindowSize() const
{
    return m_matcher->getSpeckleWindowSize();
}

void GMProcessor::setSpeckleWindowSize( const int speckleWindowSize )
{
    m_matcher->setSpeckleWindowSize( speckleWindowSize );
}

int GMProcessor::getSpeckleRange() const
{
    return m_matcher->getSpeckleRange();
}

void GMProcessor::setSpeckleRange( const int speckleRange )
{
    m_matcher->setSpeckleRange( speckleRange );
}

int GMProcessor::getDisp12MaxDiff() const
{
    return m_matcher->getDisp12MaxDiff();
}

void GMProcessor::setDisp12MaxDiff( const int disp12MaxDiff )
{
    m_matcher->setDisp12MaxDiff( disp12MaxDiff );
}

int GMProcessor::getPreFilterCap() const
{
    return m_matcher->getPreFilterCap();
}

void GMProcessor::setPreFilterCap( const int preFilterCap )
{
    m_matcher->setPreFilterCap( preFilterCap );
}

int GMProcessor::getUniquenessRatio() const
{
    return m_matcher->getUniquenessRatio();
}

void GMProcessor::setUniquenessRatio( const int uniquenessRatio )
{
    m_matcher->setUniquenessRatio( uniquenessRatio );
}

int GMProcessor::getP1() const
{
    return m_matcher->getP1();
}

void GMProcessor::setP1(int p1)
{
    m_matcher->setP1( p1 );
}

int GMProcessor::getP2() const
{
    return m_matcher->getP2();
}

void GMProcessor::setP2(int p2)
{
    m_matcher->setP2( p2 );
}

cv::Mat GMProcessor::processDisparity( const CvImage &left, const CvImage &right )
{
    CvImage leftGray;
    CvImage rightGray;

    cv::cvtColor( left,  leftGray,  cv::COLOR_BGR2GRAY );
    cv::cvtColor( right, rightGray, cv::COLOR_BGR2GRAY );

    cv::Mat leftDisp;
    cv::Mat rightDisp;

    m_matcher->compute( leftGray, rightGray, leftDisp );

/*    auto rightMatcher = cv::ximgproc::createRightMatcher(m_matcher);
    rightMatcher->compute( rightGray, leftGray, rightDisp );*/

    return leftDisp;

}
