#include "precompiled.h"

#include "disparitypreviewwidget.h"

#include "imagewidget.h"
#include "disparitycontrolwidget.h"

#include <opencv2/ximgproc.hpp>

#include <QVTKWidget.h>

#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSet.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include <vtkNew.h>
#include <vtkGenericOpenGLRenderWindow.h>

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

    m_rectifyView = new ImageWidget( this );
    m_disparityView = new ImageWidget( this );
    m_filteredDisparityView = new ImageWidget( this );
    m_controlWidget = new DisparityControlWidget( this );

    layout->addWidget( m_rectifyView, 0, 0 );
    layout->addWidget( m_disparityView, 0, 1 );
    layout->addWidget( m_filteredDisparityView, 1, 0 );
    layout->addWidget( m_controlWidget, 1, 1 );

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

int DisparityPreviewWidget::disp12MaxDiff() const
{
    return m_controlWidget->disp12MaxDiff();
}

int DisparityPreviewWidget::smallerBlockSize() const
{
    return m_controlWidget->smallerBlockSize();
}

int DisparityPreviewWidget::filterLambda() const
{
    return m_controlWidget->filterLambda();
}

int DisparityPreviewWidget::lrcThresh() const
{
    return m_controlWidget->lrcThresh();
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

void DisparityPreviewWidget::setDisp12MaxDiff( const int value )
{
    m_controlWidget->setDisp12MaxDiff( value );
}

void DisparityPreviewWidget::setSmallerBlockSize( const int value )
{
    m_controlWidget->setSmallerBlockSize( value );
}

void DisparityPreviewWidget::setFilterLambda( const int value )
{
    m_controlWidget->setFilterLambda( value );
}

void DisparityPreviewWidget::setLrcThresh( const int value ) const
{
    m_controlWidget->setLrcThresh( value );
}

void DisparityPreviewWidget::timerEvent( QTimerEvent * )
{
    if ( m_leftCapture.isOpened() && m_rightCapture.isOpened() ) {
        CvImage leftFrame;
        CvImage rightFrame;

        m_leftCapture >> leftFrame;
        m_rightCapture >> rightFrame;

        if ( !leftFrame.empty() && !rightFrame.empty() ) {
            cv::Size imageSize = CvSize( m_leftCapture.get( cv::CAP_PROP_FRAME_WIDTH ), m_leftCapture.get( cv::CAP_PROP_FRAME_HEIGHT ) );

            static cv::Mat leftCameraMatrix = ( cv::Mat1d(3, 3) << 712.8176404568455, 0, 282.7442215535847, 0, 706.5428125641499, 208.9831166091617, 0, 0, 1 );
            static cv::Mat rightCameraMatrix = ( cv::Mat1d(3, 3) << 676.621027162288, 0, 290.1575139463626, 0, 671.4547484306601, 216.9412795955598, 0, 0, 1 );

            static cv::Mat leftDistCoefficients = ( cv::Mat1d(5, 1) << 0.1274613500773498, -0.2392634803151839, -0.0178586870235027, -0.01409163435720991, 0 );
            static cv::Mat rightDistCoefficients = ( cv::Mat1d(5, 1) << 0.07029276642896512, -0.0811986780559755, -0.01571869264982598, -0.01095459372093671, 0 );

            static cv::Mat R = ( cv::Mat1d(3, 3) << 0.99981, 0.000925433, -0.0194649, -0.000405768, 0.999644, 0.0266846, 0.0194827, -0.0266716, 0.999454 );
            static cv::Mat T = ( cv::Mat1d(3, 1) << 2.35963, 0.0248091, -0.0961474 );

            cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

            cv::Mat R1, R2, P1, P2, Q;

            cv::Rect leftValidROI;
            cv::Rect rightValidROI;

            cv::stereoRectify( leftCameraMatrix, leftDistCoefficients, rightCameraMatrix, rightDistCoefficients, imageSize,
                               R, T, R1, R2, P1, P2, Q,
                               0, -1, cv::Size(), &leftValidROI, &rightValidROI );

            cv::initUndistortRectifyMap( leftCameraMatrix, leftDistCoefficients, R1, P1, imageSize, CV_32F, leftRMap, leftDMap );
            cv::initUndistortRectifyMap( rightCameraMatrix, rightDistCoefficients, R2, P2, imageSize, CV_32F, rightRMap, rightDMap );

            CvImage leftFlipped;
            CvImage rightFlipped;

            cv::Mat leftRectifiedImage;
            cv::Mat rightRectifiedImage;

            cv::remap( leftFrame, leftRectifiedImage, leftRMap, leftDMap, cv::INTER_LINEAR );
            cv::remap( rightFrame, rightRectifiedImage, rightRMap, rightDMap, cv::INTER_LINEAR );

            cv::flip( leftRectifiedImage, leftFlipped, -1 );
            cv::flip( rightRectifiedImage, rightFlipped, -1 );

            CvImage rectifyImage( leftFlipped.width() + rightFlipped.width(),
                                  std::max( leftFlipped.height(), rightFlipped.height() ),
                                  leftFlipped.type() );

            leftFlipped.copyTo( rectifyImage( cv::Rect( 0, 0, leftFlipped.width(), leftFlipped.height() ) ) );
            rightFlipped.copyTo( rectifyImage( cv::Rect( rectifyImage.width() - rightFlipped.width(),
                                                         rectifyImage.height() - rightFlipped.height(),
                                                         rightFlipped.width(), rightFlipped.height() ) ) );

            for ( auto i = 0; i < rectifyImage.height(); i+= 20 )
                cv::line( rectifyImage, cv::Point( 0, i ), cv::Point( rectifyImage.width(), i ), cv::Scalar( 255, 255, 255, 150 ) );

            m_rectifyView->setImage( rectifyImage );

            cv::imwrite( "left.png", leftFlipped );
            cv::imwrite( "right.png", rightFlipped );

            cv::Ptr< cv::StereoSGBM > left_matcher = cv::StereoSGBM::create();

            left_matcher->setBlockSize( sadWindowSize() );
            left_matcher->setMinDisparity( minDisparity() );
            left_matcher->setNumDisparities( numDisparities() );
            //left_matcher->setPreFilterSize( prefilterSize() );
            left_matcher->setPreFilterCap( prefilterCap() );
            //left_matcher->setTextureThreshold( textureThreshold() );
            left_matcher->setUniquenessRatio( uniquessRatio() );
            left_matcher->setSpeckleWindowSize( speckleWindowSize() );
            left_matcher->setSpeckleRange( speckleRange() );
            left_matcher->setDisp12MaxDiff( disp12MaxDiff() );
            //left_matcher->setSmallerBlockSize( smallerBlockSize() );

            // cv::Ptr< cv::ximgproc::DisparityWLSFilter > wls_filter = cv::ximgproc::createDisparityWLSFilter( left_matcher );

            cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher( left_matcher );

            auto left_for_matcher  = leftFlipped.clone();
            auto right_for_matcher = rightFlipped.clone();

            // cv::cvtColor( left_for_matcher,  left_for_matcher,  cv::COLOR_BGR2GRAY );
            // cv::cvtColor( right_for_matcher, right_for_matcher, cv::COLOR_BGR2GRAY );

            CvImage left_disp, right_disp;

            left_matcher->compute( left_for_matcher, right_for_matcher, left_disp );
            right_matcher->compute( right_for_matcher,left_for_matcher, right_disp );

            cv::Mat filtered_disp;

            filtered_disp = left_disp;

           /* wls_filter->setLambda( filterLambda() );
            wls_filter->setSigmaColor( 0.8 );
            wls_filter->setLRCthresh( lrcThresh() );
            wls_filter->filter( left_disp, leftFrame , filtered_disp, right_disp );*/

            cv::Mat raw_disp_vis;
            cv::ximgproc::getDisparityVis( left_disp, raw_disp_vis, 1.0 );

            double min;
            double max;
            cv::minMaxIdx(raw_disp_vis, &min, &max);

            cv::Mat cm_disp, scaledDisparityMap;
            cv::convertScaleAbs( raw_disp_vis, scaledDisparityMap, 255 / ( max - min ) );
            cv::applyColorMap( scaledDisparityMap, cm_disp, cv::COLORMAP_JET );

            m_disparityView->setImage( cm_disp );

            cv::Mat raw_disp_vis2;
            cv::ximgproc::getDisparityVis( filtered_disp, raw_disp_vis2, 1.0 );

            double min2;
            double max2;
            cv::minMaxIdx(raw_disp_vis2, &min2, &max2);

            cv::Mat cm_disp2, scaledDisparityMap2;
            cv::convertScaleAbs( raw_disp_vis2, scaledDisparityMap2, 255 / ( max2 - min2 ) );
            cv::applyColorMap( scaledDisparityMap2, cm_disp2, cv::COLORMAP_JET );

            m_filteredDisparityView->setImage( scaledDisparityMap2 );

        }

    }

}

// PreviewWidget
PreviewWidget::PreviewWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize( leftCameraIndex, rightCameraIndex );
}

void PreviewWidget::initialize( const int leftCameraIndex, const int rightCameraIndex )
{
    QFile cssFile(":/resources/qss/style.css");

    if ( cssFile.open( QIODevice::ReadOnly ) ) {
        QString cssString( cssFile.readAll() );
        setStyleSheet( cssString );
    }
    else
        QMessageBox::critical( nullptr, tr( "Error"), tr( "Can't load css file:" ) + cssFile.fileName() );

    m_disparityWidget = new DisparityPreviewWidget( leftCameraIndex, rightCameraIndex, this );
    m_3dWidget = new QVTKWidget( this );
    m_3dWidget->resize(200, 200);

    addWidget( m_disparityWidget );
    addWidget( m_3dWidget );

   // m_visualizer = new pcl::visualization::PCLVisualizer( "viewer" );
 //   m_3dWidget->SetRenderWindow( m_visualizer->getRenderWindow() );
}
