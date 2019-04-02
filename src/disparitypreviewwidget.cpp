#include "precompiled.h"

#include "disparitypreviewwidget.h"

#include "imagewidget.h"
#include "disparitycontrolwidget.h"

#include <opencv2/ximgproc.hpp>


// DisparityPreviewWidget
DisparityPreviewWidget::DisparityPreviewWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent )
    : QWidget( parent )
{
    initialize( leftCameraIndex, rightCameraIndex );
}

void DisparityPreviewWidget::initialize( const int leftCameraIndex, const int rightCameraIndex )
{
    m_leftCapture.open( leftCameraIndex );
    m_rightCapture.open( rightCameraIndex );

    auto layout = new QGridLayout( this );

    m_leftView = new ImageWidget( this );
    m_rightView = new ImageWidget( this );
    m_disparityView = new ImageWidget( this );
    m_filteredDisparityView = new ImageWidget( this );
    m_controlWidget = new DisparityControlWidget( this );

    layout->addWidget( m_leftView, 0, 0 );
    layout->addWidget( m_rightView, 0, 1 );
    layout->addWidget( m_disparityView, 1, 0 );
    layout->addWidget( m_filteredDisparityView, 1, 1 );
    layout->addWidget( m_controlWidget, 2, 0 );

    startTimer( 1 );

}

int DisparityPreviewWidget::prefilterSize() const
{
    return m_controlWidget->prefilterSize();
}

int DisparityPreviewWidget::prefilterCap() const
{
    return m_controlWidget->prefilterCap();
}

int DisparityPreviewWidget::sadWindowSize() const
{
    return m_controlWidget->sadWindowSize();
}

int DisparityPreviewWidget::minDisparity() const
{
    return m_controlWidget->minDisparity();
}

int DisparityPreviewWidget::numDisparities() const
{
    return m_controlWidget->numDisparities();
}

int DisparityPreviewWidget::textureThreshold() const
{
    return m_controlWidget->textureThreshold();
}

int DisparityPreviewWidget::uniquessRatio() const
{
    return m_controlWidget->uniquessRatio();
}

int DisparityPreviewWidget::speckleWindowSize() const
{
    return m_controlWidget->speckleWindowSize();
}

int DisparityPreviewWidget::speckleRange() const
{
    return m_controlWidget->speckleRange();
}

void DisparityPreviewWidget::setPrefilterSize( const int value )
{
    m_controlWidget->setPrefilterSize( value );
}

void DisparityPreviewWidget::setPrefilterCap( const int value )
{
    m_controlWidget->setPrefilterCap( value );
}

void DisparityPreviewWidget::setSadWindowSize( const int value )
{
    m_controlWidget->setSadWindowSize( value );
}

void DisparityPreviewWidget::setMinDisparity( const int value )
{
    m_controlWidget->setMinDisparity( value );
}

void DisparityPreviewWidget::setNumDisparities( const int value )
{
    m_controlWidget->setNumDisparities( value );
}

void DisparityPreviewWidget::setTextureThreshold( const int value )
{
    m_controlWidget->setTextureThreshold( value );
}

void DisparityPreviewWidget::setUniquessRatio( const int value )
{
    m_controlWidget->setUniquessRatio( value );
}

void DisparityPreviewWidget::setSpeckleWindowSize( const int value )
{
    m_controlWidget->setSpeckleWindowSize( value );
}

void DisparityPreviewWidget::setSpeckleRange( const int value )
{
    m_controlWidget->setSpeckleRange( value );
}

void DisparityPreviewWidget::timerEvent(QTimerEvent *event)
{
    if ( m_leftCapture.isOpened() && m_rightCapture.isOpened() ) {
        CvImage leftFrame;
        CvImage rightFrame;

        m_leftCapture >> leftFrame;
        m_rightCapture >> rightFrame;

        if ( !leftFrame.empty() && !rightFrame.empty() ) {
            cv::Size imageSize = CvSize( m_leftCapture.get( cv::CAP_PROP_FRAME_WIDTH ), m_leftCapture.get( cv::CAP_PROP_FRAME_HEIGHT ) );

            static cv::Mat leftCameraMatrix = ( cv::Mat1d(3, 3) << 624.847, 0, 308.76, 0, 624.898, 246.337, 0, 0, 1 );
            static cv::Mat rightCameraMatrix = ( cv::Mat1d(3, 3) << 637.909, 0, 311.316, 0, 636.603, 241.16, 0, 0, 1 );

            static cv::Mat leftDistCoefficients = ( cv::Mat1d(5, 1) << 0.0664186, -0.203946, -0.00328787, -0.00120171, 0.219019 );
            static cv::Mat rightDistCoefficients = ( cv::Mat1d(5, 1) << 0.0673875, -0.220073, -0.00332591, -0.000892592, 0.222464 );

            static cv::Mat R = ( cv::Mat1d(3, 3) << 0.999419, -0.00203004, -0.0340237, 0.00329704, 0.999302, 0.0372242, 0.0339244, -0.0373147, 0.998728 );
            static cv::Mat T = ( cv::Mat1d(3, 1) << 9.57281, -0.0293154, 0.785558 );

            cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

            cv::Mat R1, R2, P1, P2, Q;

            cv::Rect leftValidROI;
            cv::Rect rightValidROI;

            cv::stereoRectify( leftCameraMatrix, leftDistCoefficients, rightCameraMatrix, rightDistCoefficients, imageSize,
                               R, T, R1, R2, P1, P2, Q,
                               cv::CALIB_ZERO_DISPARITY, 1, imageSize, &leftValidROI, &rightValidROI );

            cv::initUndistortRectifyMap( leftCameraMatrix, leftDistCoefficients, R1, P1, imageSize, CV_16SC2, leftRMap, leftDMap );
            cv::initUndistortRectifyMap( rightCameraMatrix, rightDistCoefficients, R2, P2, imageSize, CV_16SC2, rightRMap, rightDMap );

            cv::Mat leftFlipped;
            cv::Mat rightFlipped;

            cv::Mat leftRectifiedImage;
            cv::Mat rightRectifiedImage;

            cv::remap( leftFrame, leftRectifiedImage, leftRMap, leftDMap, cv::INTER_LANCZOS4 );
            cv::remap( rightFrame, rightRectifiedImage, rightRMap, rightDMap, cv::INTER_LANCZOS4 );

            cv::flip( leftRectifiedImage, leftFlipped, -1 );
            cv::flip( rightRectifiedImage, rightFlipped, -1 );

            m_leftView->setImage( leftFlipped );
            m_rightView->setImage( rightFlipped );


            cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create();
            cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter( left_matcher );

            left_matcher->setBlockSize( sadWindowSize() );
            left_matcher->setMinDisparity( minDisparity() );
            left_matcher->setNumDisparities( numDisparities() );
            // left_matcher->setPreFilterSize( prefilterSize() );
            left_matcher->setPreFilterCap( prefilterCap() );
            // left_matcher->setTextureThreshold( textureThreshold() );
            left_matcher->setUniquenessRatio( uniquessRatio() );
            left_matcher->setSpeckleWindowSize( speckleWindowSize() );
            left_matcher->setSpeckleRange( speckleRange() );

            cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher( left_matcher );

            auto left_for_matcher  = leftFlipped.clone();
            auto right_for_matcher = rightFlipped.clone();

            cv::cvtColor( left_for_matcher,  left_for_matcher,  cv::COLOR_BGR2GRAY );
            cv::cvtColor( right_for_matcher, right_for_matcher, cv::COLOR_BGR2GRAY );

            CvImage left_disp, right_disp;

            left_matcher->compute( left_for_matcher, right_for_matcher, left_disp );
            right_matcher->compute( right_for_matcher,left_for_matcher, right_disp );

           // cv::Mat filtered_disp;

           /* wls_filter->setLambda( 10000 );
            wls_filter->setSigmaColor( 1.2 );
            wls_filter->filter( left_disp, leftFrame , filtered_disp, right_disp );*/

            cv::Mat raw_disp_vis;
            cv::ximgproc::getDisparityVis( left_disp, raw_disp_vis, 1.0 );

            double min;
            double max;
            cv::minMaxIdx(raw_disp_vis, &min, &max);

            cv::Mat cm_disp, scaledDisparityMap;
            cv::convertScaleAbs( raw_disp_vis, scaledDisparityMap, 255 / ( max - min ) );
            cv::applyColorMap( scaledDisparityMap, cm_disp, cv::COLORMAP_JET );

            m_disparityView->setImage( scaledDisparityMap );

            /*cv::Mat raw_disp_vis2;
            cv::ximgproc::getDisparityVis( filtered_disp, raw_disp_vis2, 1.0 );

            double min2;
            double max2;
            cv::minMaxIdx(raw_disp_vis2, &min2, &max2);

            cv::Mat cm_disp2, scaledDisparityMap2;
            cv::convertScaleAbs( raw_disp_vis2, scaledDisparityMap2, 255 / ( max2 - min2 ) );
            cv::applyColorMap( scaledDisparityMap2, cm_disp2, cv::COLORMAP_JET );

            m_filteredDisparityView->setImage( scaledDisparityMap2 );*/

        }

    }

}

