#include "src/common/precompiled.h"

#include "disparitypreviewwidget.h"

#include "src/common/imagewidget.h"
#include "disparitycontrolwidget.h"

#include <opencv2/ximgproc.hpp>

#include "pclwidget.h"

#include "application.h"

#include "src/common/functions.h"

DisparityPreviewWidget::DisparityPreviewWidget( QWidget* parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void DisparityPreviewWidget::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );

    m_rectifyView = new ImageWidget( this );
    m_disparityView = new ImageWidget( this );

    addWidget( m_rectifyView );
    addWidget( m_disparityView );

}

ImageWidget *DisparityPreviewWidget::rectifyView() const
{
    return m_rectifyView;
}

ImageWidget *DisparityPreviewWidget::disparityView() const
{
    return m_disparityView;
}

// PreviewWidget
PreviewWidget::PreviewWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

PreviewWidget::~PreviewWidget()
{
    m_leftCamera->Close();
    m_rightCamera->Close();
}

void PreviewWidget::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    auto app = application();

    checkVimbaStatus( app->vimbaSystem().OpenCameraByID( leftCameraIp.c_str(), VmbAccessModeFull, m_leftCamera ),
        "Could not start open camera; ip = " + leftCameraIp );

    checkVimbaStatus( app->vimbaSystem().OpenCameraByID( rightCameraIp.c_str(), VmbAccessModeFull, m_rightCamera ),
        "Could not start open camera; ip = " + rightCameraIp );

    setVimbaFeature( m_leftCamera, "PixelFormat", VmbPixelFormatBgr8 );
    setVimbaFeature( m_rightCamera, "PixelFormat", VmbPixelFormatBgr8 );

    m_view = new DisparityPreviewWidget( this );
    m_view->resize(800, 800);

    m_controlWidget = new DisparityControlWidget( this );
    m_3dWidget = new PCLViewer( this );

    addWidget( m_view );
    addWidget( m_controlWidget );
    addWidget( m_3dWidget );

    startTimer( 1 );

}

BMControlWidget *PreviewWidget::bmControlWidget() const
{
    return m_controlWidget->bmControlWidget();
}

void PreviewWidget::setDecimation( const VimbaDecimationType type )
{
    int decimationFactor = static_cast<int>( type );

    setVimbaFeature( m_leftCamera, "DecimationHorizontal", decimationFactor );
    setVimbaFeature( m_leftCamera, "DecimationVertical", decimationFactor );
    setVimbaFeature( m_leftCamera, "OffsetX", 0 );
    setVimbaFeature( m_leftCamera, "OffsetY", 0 );
    setVimbaFeature( m_leftCamera, "Width", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );
    setVimbaFeature( m_leftCamera, "Height", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );

    setVimbaFeature( m_rightCamera, "DecimationHorizontal", decimationFactor );
    setVimbaFeature( m_rightCamera, "DecimationVertical", decimationFactor );
    setVimbaFeature( m_rightCamera, "OffsetX", 0 );
    setVimbaFeature( m_rightCamera, "OffsetY", 0 );
    setVimbaFeature( m_rightCamera, "Width", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );
    setVimbaFeature( m_rightCamera, "Height", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );

}

bool PreviewWidget::loadCalibrationFile( const std::string &fileName )
{
    m_calibration.loadYaml( fileName );
}

void PreviewWidget::timerEvent( QTimerEvent * )
{
    CvImage leftFrame;
    CvImage rightFrame;

    AVT::VmbAPI::FramePtr leftPFrame;
    AVT::VmbAPI::FramePtr rightPFrame;
    VmbUchar_t *leftPImage;
    VmbUchar_t *rightPImage;
    VmbUint32_t timeout = 500;
    VmbUint32_t nWidth = 0;
    VmbUint32_t nHeight = 0;

    VmbFrameStatusType status;

    checkVimbaStatus( m_leftCamera->AcquireSingleImage( leftPFrame, timeout ), "FAILED to aquire frame!" );
    checkVimbaStatus( m_rightCamera->AcquireSingleImage( rightPFrame, timeout ), "FAILED to aquire frame!" );

    checkVimbaStatus( leftPFrame->GetReceiveStatus( status ), "FAILED to aquire frame!" );
    checkVimbaStatus( rightPFrame->GetReceiveStatus( status ), "FAILED to aquire frame!" );

    checkVimbaStatus( leftPFrame->GetWidth( nWidth ), "FAILED to aquire width of frame!" );
    checkVimbaStatus( rightPFrame->GetWidth( nWidth ), "FAILED to aquire width of frame!" );

    checkVimbaStatus( leftPFrame->GetHeight( nHeight ), "FAILED to aquire height of frame!" );
    checkVimbaStatus( rightPFrame->GetHeight( nHeight ), "FAILED to aquire height of frame!" );

    checkVimbaStatus( leftPFrame->GetImage( leftPImage ), "FAILED to acquire image data of frame!" );
    checkVimbaStatus( rightPFrame->GetImage( rightPImage ), "FAILED to acquire image data of frame!" );

    auto leftMat = cv::Mat( nHeight, nWidth, CV_8UC3, leftPImage );
    leftMat.copyTo( leftFrame );

    auto rightMat = cv::Mat( nHeight, nWidth, CV_8UC3, rightPImage );
    rightMat.copyTo( rightFrame );

    if ( !leftFrame.empty() && !rightFrame.empty() ) {

        if ( m_calibration.isOk() ) {

            if ( m_calibration.leftCameraResults().frameSize() != m_calibration.rightCameraResults().frameSize() )
                throw std::exception();

            CvImage leftFlipped;
            CvImage rightFlipped;

            cv::Mat leftRectifiedImage;
            cv::Mat rightRectifiedImage;

            cv::remap( leftFrame, leftRectifiedImage, m_calibration.leftRMap(), m_calibration.leftDMap(), cv::INTER_LINEAR );
            cv::remap( rightFrame, rightRectifiedImage, m_calibration.rightRMap(), m_calibration.rightDMap(), cv::INTER_LINEAR );

            cv::flip( leftRectifiedImage, leftFlipped, -1 );
            cv::flip( rightRectifiedImage, rightFlipped, -1 );

            cv::Ptr< cv::StereoBM > matcher = cv::StereoBM::create();

            matcher->setBlockSize( m_controlWidget->bmControlWidget()->sadWindowSize() );
            matcher->setMinDisparity( m_controlWidget->bmControlWidget()->minDisparity() );
            matcher->setNumDisparities( m_controlWidget->bmControlWidget()->numDisparities() );
            matcher->setPreFilterSize( m_controlWidget->bmControlWidget()->prefilterSize() );
            matcher->setPreFilterCap( m_controlWidget->bmControlWidget()->prefilterCap() );
            matcher->setTextureThreshold( m_controlWidget->bmControlWidget()->textureThreshold() );
            matcher->setUniquenessRatio( m_controlWidget->bmControlWidget()->uniquessRatio() );
            matcher->setSpeckleWindowSize( m_controlWidget->bmControlWidget()->speckleWindowSize() );
            matcher->setSpeckleRange( m_controlWidget->bmControlWidget()->speckleRange() );
            matcher->setDisp12MaxDiff( m_controlWidget->bmControlWidget()->disp12MaxDiff() );
            matcher->setSmallerBlockSize( m_controlWidget->bmControlWidget()->smallerBlockSize() );

            CvImage left_for_matcher;
            CvImage right_for_matcher;

            cv::cvtColor( leftFlipped,  left_for_matcher,  cv::COLOR_BGR2GRAY );
            cv::cvtColor( rightFlipped, right_for_matcher, cv::COLOR_BGR2GRAY );

            cv::Mat disp;

            matcher->compute( left_for_matcher, right_for_matcher, disp );

            double min, max;

            cv::minMaxIdx( disp, &min, &max );
            double multiplier = 255.0 / (max - min);

            cv::Mat converted;

            disp.convertTo( converted, CV_8U, multiplier, -min * multiplier );

            cv::Mat colored;

            cv::applyColorMap( converted, colored, cv::COLORMAP_JET );

            m_view->disparityView()->setImage( colored );

            CvImage rectifyImage( leftFlipped.width() + rightFlipped.width(),
                                  std::max( leftFlipped.height(), rightFlipped.height() ),
                                  leftFlipped.type() );

            leftFlipped.copyTo( rectifyImage( cv::Rect( 0, 0, leftFlipped.width(), leftFlipped.height() ) ) );
            rightFlipped.copyTo( rectifyImage( cv::Rect( rectifyImage.width() - rightFlipped.width(),
                                                         rectifyImage.height() - rightFlipped.height(),
                                                         rightFlipped.width(), rightFlipped.height() ) ) );

            drawTraceLines( rectifyImage, 15 );

            m_view->rectifyView()->setImage( rectifyImage );


        }

    }

}

