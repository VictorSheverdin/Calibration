#include "src/common/precompiled.h"

#include "disparitypreviewwidget.h"

#include "src/common/imagewidget.h"
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

#include "application.h"

#include "src/common/functions.h"

// DisparityPreviewWidget
DisparityPreviewWidget::DisparityPreviewWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent )
    : QWidget( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void DisparityPreviewWidget::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    auto app = application();

    checkVimbaStatus( app->vimbaSystem().OpenCameraByID( leftCameraIp.c_str(), VmbAccessModeFull, m_leftCamera ),
        "Could not start open camera; ip = " + leftCameraIp );

    checkVimbaStatus( app->vimbaSystem().OpenCameraByID( rightCameraIp.c_str(), VmbAccessModeFull, m_rightCamera ),
        "Could not start open camera; ip = " + rightCameraIp );

    setVimbaFeature( m_leftCamera, "PixelFormat", VmbPixelFormatBgr8 );
    setVimbaFeature( m_rightCamera, "PixelFormat", VmbPixelFormatBgr8 );

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

void DisparityPreviewWidget::setDecimation( const VimbaDecimationType type )
{
    int decimationFactor = static_cast<int>( type );

    setVimbaFeature( m_leftCamera, "DecimationHorizontal",  decimationFactor );
    setVimbaFeature( m_leftCamera, "DecimationVertical",  decimationFactor );
    setVimbaFeature( m_leftCamera, "OffsetX",  0 );
    setVimbaFeature( m_leftCamera, "OffsetY",  0 );
    setVimbaFeature( m_leftCamera, "Width", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );
    setVimbaFeature( m_leftCamera, "Height", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );

    setVimbaFeature( m_rightCamera, "DecimationHorizontal",  decimationFactor );
    setVimbaFeature( m_rightCamera, "DecimationVertical",  decimationFactor );
    setVimbaFeature( m_rightCamera, "OffsetX",  0 );
    setVimbaFeature( m_rightCamera, "OffsetY",  0 );
    setVimbaFeature( m_rightCamera, "Width", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );
    setVimbaFeature( m_rightCamera, "Height", VIMBA_ORIGINAL_FRAME_SIZE / decimationFactor );

}

bool DisparityPreviewWidget::loadCalibrationFile( const std::string &fileName )
{
    m_calibration.loadYaml( fileName );

    cv::initUndistortRectifyMap( m_calibration.leftCameraResults().cameraMatrix(), m_calibration.leftCameraResults().distortionCoefficients(),
                                 m_calibration.leftRectifyMatrix(), m_calibration.leftProjectionMatrix(), m_calibration.leftCameraResults().frameSize(),
                                 CV_32FC2, leftRMap, leftDMap );

    cv::initUndistortRectifyMap( m_calibration.rightCameraResults().cameraMatrix(), m_calibration.rightCameraResults().distortionCoefficients(),
                                 m_calibration.rightRectifyMatrix(), m_calibration.rightProjectionMatrix(), m_calibration.leftCameraResults().frameSize(),
                                 CV_32FC2, rightRMap, rightDMap );

}

void DisparityPreviewWidget::timerEvent( QTimerEvent * )
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

            cv::remap( leftFrame, leftRectifiedImage, leftRMap, leftDMap, cv::INTER_LINEAR );
            cv::remap( rightFrame, rightRectifiedImage, rightRMap, rightDMap, cv::INTER_LINEAR );

            cv::flip( leftRectifiedImage, leftFlipped, -1 );
            cv::flip( rightRectifiedImage, rightFlipped, -1 );

            cv::Ptr< cv::StereoBM > matcher = cv::StereoBM::create();

            matcher->setBlockSize( sadWindowSize() );
            matcher->setMinDisparity( minDisparity() );
            matcher->setNumDisparities( numDisparities() );
            matcher->setPreFilterSize( prefilterSize() );
            matcher->setPreFilterCap( prefilterCap() );
            matcher->setTextureThreshold( textureThreshold() );
            matcher->setUniquenessRatio( uniquessRatio() );
            matcher->setSpeckleWindowSize( speckleWindowSize() );
            matcher->setSpeckleRange( speckleRange() );
            matcher->setDisp12MaxDiff( disp12MaxDiff() );
            matcher->setSmallerBlockSize( smallerBlockSize() );

            CvImage left_for_matcher;
            CvImage right_for_matcher;

            cv::cvtColor( leftFlipped,  left_for_matcher,  cv::COLOR_BGR2GRAY );
            cv::cvtColor( rightFlipped, right_for_matcher, cv::COLOR_BGR2GRAY );

            cv::Mat disp;

            matcher->compute( left_for_matcher, right_for_matcher, disp );

            double min, max;

            cv::minMaxIdx( disp, &min, &max );
            double multiplier = 255.0 / (max - min);

            CvImage coloredDisp( disp.rows, disp.cols, leftFrame.type() );

            for ( auto i = 0; i < disp.rows; ++i )
                for ( auto j = 0; j < disp.cols; ++j )
                    coloredDisp.at<cv::Scalar>(i, j) = cv::Scalar( 0, 0, 0 );

            m_disparityView->setImage( coloredDisp );

            m_filteredDisparityView->setImage( coloredDisp );

            //cv::imwrite( "/home/sheverdin/test.jpg", disp );

            CvImage rectifyImage( leftFlipped.width() + rightFlipped.width(),
                                  std::max( leftFlipped.height(), rightFlipped.height() ),
                                  leftFlipped.type() );

            leftFlipped.copyTo( rectifyImage( cv::Rect( 0, 0, leftFlipped.width(), leftFlipped.height() ) ) );
            rightFlipped.copyTo( rectifyImage( cv::Rect( rectifyImage.width() - rightFlipped.width(),
                                                         rectifyImage.height() - rightFlipped.height(),
                                                         rightFlipped.width(), rightFlipped.height() ) ) );

            drawTraceLines( rectifyImage, 15 );

            m_rectifyView->setImage( rectifyImage );


        }

    }

}

// PreviewWidget
PreviewWidget::PreviewWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void PreviewWidget::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    QFile cssFile(":/resources/qss/style.css");

    if ( cssFile.open( QIODevice::ReadOnly ) ) {
        QString cssString( cssFile.readAll() );
        setStyleSheet( cssString );
    }
    else
        QMessageBox::critical( nullptr, tr( "Error"), tr( "Can't load css file:" ) + cssFile.fileName() );

    m_disparityWidget = new DisparityPreviewWidget( leftCameraIp, rightCameraIp, this );
    m_3dWidget = new QVTKWidget( this );
    m_3dWidget->resize(200, 200);

    addWidget( m_disparityWidget );
    addWidget( m_3dWidget );

   // m_visualizer = new pcl::visualization::PCLVisualizer( "viewer" );
 //   m_3dWidget->SetRenderWindow( m_visualizer->getRenderWindow() );
}

bool PreviewWidget::loadCalibrationFile( const std::string &fileName )
{
    m_disparityWidget->loadCalibrationFile( fileName );
}
