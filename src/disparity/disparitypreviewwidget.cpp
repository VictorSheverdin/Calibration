#include "src/common/precompiled.h"

#include "disparitypreviewwidget.h"

#include "src/common/imagewidget.h"
#include "disparitycontrolwidget.h"

#include <opencv2/ximgproc.hpp>

#include "pclwidget.h"

#include "application.h"

#include "src/common/functions.h"

#include <pcl/io/ply_io.h>

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

    auto tabWidget = new QTabWidget( this );

    m_view = new DisparityPreviewWidget( this );
    tabWidget->resize(1200, 800);
    m_3dWidget = new PCLViewer( this );

    tabWidget->addTab( m_view, tr( "Disparity" ) );
    tabWidget->addTab( m_3dWidget, tr( "3D priview" ) );

    m_controlWidget = new DisparityControlWidget( this );

    addWidget( tabWidget );
    addWidget( m_controlWidget );

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

            cv::Mat leftRectifiedImage;
            cv::Mat rightRectifiedImage;

            cv::remap( leftFrame, leftRectifiedImage, m_calibration.leftRMap(), m_calibration.leftDMap(), cv::INTER_LINEAR );
            cv::remap( rightFrame, rightRectifiedImage, m_calibration.rightRMap(), m_calibration.rightDMap(), cv::INTER_LINEAR );

            CvImage previewImage = stackImages( leftFrame, rightFrame );
            drawTraceLines( previewImage, 15 );
            m_view->rectifyView()->setImage( previewImage );

            cv::Mat disp;

            if ( m_controlWidget->isBmMethod() ) {
                m_bmProcessor.setBlockSize( m_controlWidget->bmControlWidget()->sadWindowSize() );
                m_bmProcessor.setMinDisparity( m_controlWidget->bmControlWidget()->minDisparity() );
                m_bmProcessor.setNumDisparities( m_controlWidget->bmControlWidget()->numDisparities() );
                m_bmProcessor.setPreFilterSize( m_controlWidget->bmControlWidget()->prefilterSize() );
                m_bmProcessor.setPreFilterCap( m_controlWidget->bmControlWidget()->prefilterCap() );
                m_bmProcessor.setTextureThreshold( m_controlWidget->bmControlWidget()->textureThreshold() );
                m_bmProcessor.setUniquenessRatio( m_controlWidget->bmControlWidget()->uniquessRatio() );
                m_bmProcessor.setSpeckleWindowSize( m_controlWidget->bmControlWidget()->speckleWindowSize() );
                m_bmProcessor.setSpeckleRange( m_controlWidget->bmControlWidget()->speckleRange() );
                m_bmProcessor.setDisp12MaxDiff( m_controlWidget->bmControlWidget()->disp12MaxDiff() );

                disp = m_bmProcessor.processDisparity( leftFrame, rightFrame );
            }
            else if ( m_controlWidget->isGmMethod() ) {
                m_gmProcessor.setMode( m_controlWidget->gmControlWidget()->mode() );
                m_gmProcessor.setPreFilterCap( m_controlWidget->gmControlWidget()->prefilterCap() );
                m_gmProcessor.setBlockSize( m_controlWidget->gmControlWidget()->sadWindowSize() );
                m_gmProcessor.setMinDisparity( m_controlWidget->gmControlWidget()->minDisparity() );
                m_gmProcessor.setNumDisparities( m_controlWidget->gmControlWidget()->numDisparities() );
                m_gmProcessor.setUniquenessRatio( m_controlWidget->gmControlWidget()->uniquessRatio() );
                m_gmProcessor.setSpeckleWindowSize( m_controlWidget->gmControlWidget()->speckleWindowSize() );
                m_gmProcessor.setSpeckleRange( m_controlWidget->gmControlWidget()->speckleRange() );
                m_gmProcessor.setDisp12MaxDiff( m_controlWidget->gmControlWidget()->disp12MaxDiff() );
                m_gmProcessor.setP1( m_controlWidget->gmControlWidget()->p1() );
                m_gmProcessor.setP2( m_controlWidget->gmControlWidget()->p2() );

                disp = m_gmProcessor.processDisparity( leftFrame, rightFrame );

            }

            m_view->disparityView()->setImage( colorizeDisparity( disp ) );

            cv::Mat points;

            cv::reprojectImageTo3D( disp, points, m_calibration.disparityToDepthMatrix(), true );

            pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud( new pcl::PointCloud< pcl::PointXYZRGB > );

            for (int rows = 0; rows < points.rows; ++rows) {
                for (int cols = 0; cols < points.cols; ++cols) {

                    cv::Point3f point = points.at< cv::Point3f >(rows, cols);

//                    if ( point.x > -10 && point.y > -10 && point.z > -10 && point.x < 10 && point.y < 10 && point.z < 10 ) {
                        pcl::PointXYZRGB pclPoint;
                        pclPoint.x = point.x;
                        pclPoint.y = point.y;
                        pclPoint.z = point.z;

                        cv::Vec3b intensity = leftFrame.at<cv::Vec3b>(rows,cols); //BGR
                        uint32_t rgb = (static_cast<uint32_t>(intensity[2]) << 16 | static_cast<uint32_t>(intensity[1]) << 8 | static_cast<uint32_t>(intensity[0]));
                        pclPoint.rgb = *reinterpret_cast<float*>(&rgb);
                        cloud->push_back( pclPoint );
//                    }

                }
            }

            m_3dWidget->pclviewer->updatePointCloud( cloud, "disparity" );
            m_3dWidget->update();

        }

    }

}

