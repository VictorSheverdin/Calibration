#include "src/common/precompiled.h"

#include "disparitypreviewwidget.h"

#include "src/common/imagewidget.h"
#include "disparitycontrolwidget.h"
#include "disparityiconswidget.h"

#include <opencv2/ximgproc.hpp>

#include "pclwidget.h"

#include "application.h"

#include "src/common/functions.h"
#include "src/common/fileslistwidget.h"

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


// DisparityWidgetBase
DisparityWidgetBase::DisparityWidgetBase( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void DisparityWidgetBase::initialize()
{
    m_bmProcessor = std::shared_ptr<BMDisparityProcessor>( new BMDisparityProcessor );
    m_gmProcessor = std::shared_ptr<GMDisparityProcessor>( new GMDisparityProcessor );

    auto tabWidget = new QTabWidget( this );

    m_view = new DisparityPreviewWidget( this );
    tabWidget->resize(1200, 800);
    m_3dWidget = new PCLViewer( this );

    tabWidget->addTab( m_view, tr( "Disparity" ) );
    tabWidget->addTab( m_3dWidget, tr( "3D priview" ) );

    m_controlWidget = new DisparityControlWidget( this );

    addWidget( tabWidget );
    addWidget( m_controlWidget );

    connect( m_controlWidget, &DisparityControlWidget::valueChanged, this, &DisparityWidgetBase::valueChanged );
}

BMControlWidget *DisparityWidgetBase::bmControlWidget() const
{
    return m_controlWidget->bmControlWidget();
}

GMControlWidget *DisparityWidgetBase::gmControlWidget() const
{
    return m_controlWidget->gmControlWidget();
}

bool DisparityWidgetBase::loadCalibrationFile( const QString &fileName )
{
    m_processor.loadYaml( fileName.toStdString() );
}

void DisparityWidgetBase::loadCalibrationDialog()
{
    auto file = QFileDialog::getOpenFileName(
                        this,
                        tr( "Select calibration file" ),
                        QString(),
                        "Calibration files (*.yaml)" );

    if ( !file.isEmpty() )
        loadCalibrationFile( file );
}

void DisparityWidgetBase::updateFrame(const CvImage leftFrame, const CvImage rightFrame)
{
     if ( !leftFrame.empty() && !rightFrame.empty() ) {

        if ( m_controlWidget->isBmMethod() ) {
            m_bmProcessor->setBlockSize( m_controlWidget->bmControlWidget()->sadWindowSize() );
            m_bmProcessor->setMinDisparity( m_controlWidget->bmControlWidget()->minDisparity() );
            m_bmProcessor->setNumDisparities( m_controlWidget->bmControlWidget()->numDisparities() );
            m_bmProcessor->setPreFilterSize( m_controlWidget->bmControlWidget()->prefilterSize() );
            m_bmProcessor->setPreFilterCap( m_controlWidget->bmControlWidget()->prefilterCap() );
            m_bmProcessor->setTextureThreshold( m_controlWidget->bmControlWidget()->textureThreshold() );
            m_bmProcessor->setUniquenessRatio( m_controlWidget->bmControlWidget()->uniquessRatio() );
            m_bmProcessor->setSpeckleWindowSize( m_controlWidget->bmControlWidget()->speckleWindowSize() );
            m_bmProcessor->setSpeckleRange( m_controlWidget->bmControlWidget()->speckleRange() );
            m_bmProcessor->setDisp12MaxDiff( m_controlWidget->bmControlWidget()->disp12MaxDiff() );

            m_processor.setDisparityProcessor( m_bmProcessor );
        }
        else if ( m_controlWidget->isGmMethod() ) {
            m_gmProcessor->setMode( m_controlWidget->gmControlWidget()->mode() );
            m_gmProcessor->setPreFilterCap( m_controlWidget->gmControlWidget()->prefilterCap() );
            m_gmProcessor->setBlockSize( m_controlWidget->gmControlWidget()->sadWindowSize() );
            m_gmProcessor->setMinDisparity( m_controlWidget->gmControlWidget()->minDisparity() );
            m_gmProcessor->setNumDisparities( m_controlWidget->gmControlWidget()->numDisparities() );
            m_gmProcessor->setUniquenessRatio( m_controlWidget->gmControlWidget()->uniquessRatio() );
            m_gmProcessor->setSpeckleWindowSize( m_controlWidget->gmControlWidget()->speckleWindowSize() );
            m_gmProcessor->setSpeckleRange( m_controlWidget->gmControlWidget()->speckleRange() );
            m_gmProcessor->setDisp12MaxDiff( m_controlWidget->gmControlWidget()->disp12MaxDiff() );
            m_gmProcessor->setP1( m_controlWidget->gmControlWidget()->p1() );
            m_gmProcessor->setP2( m_controlWidget->gmControlWidget()->p2() );

            m_processor.setDisparityProcessor( m_gmProcessor );

        }

        auto stereoResult = m_processor.process( leftFrame, rightFrame );

        m_view->rectifyView()->setImage( stereoResult.previewImage() );

        m_view->disparityView()->setImage( stereoResult.colorizedDisparity() );

        if ( stereoResult.pointCloud() )
            m_3dWidget->setPointCloud( stereoResult.pointCloud() );

        m_3dWidget->repaint();

    }

}

// CameraDisparityWidget
CameraDisparityWidget::CameraDisparityWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : DisparityWidgetBase( parent ), m_leftCam( leftCameraIp.toStdString() ), m_rightCam( rightCameraIp.toStdString() )
{
    initialize();
}

void CameraDisparityWidget::initialize()
{
    connect( &m_rightCam, &VimbaCamera::receivedFrame, this, &CameraDisparityWidget::updateFrame );

    startTimer( 1000 / 10 );

}

void CameraDisparityWidget::updateFrame()
{
    DisparityWidgetBase::updateFrame( m_leftCam.getFrame(), m_rightCam.getFrame() );
}

void CameraDisparityWidget::timerEvent( QTimerEvent * )
{
    auto &vimbaSystem = AVT::VmbAPI::VimbaSystem::GetInstance();

    setVimbaFeature( vimbaSystem, "ActionDeviceKey", ACTION_DEVICE_KEY );
    setVimbaFeature( vimbaSystem, "ActionGroupKey", ACTION_GROUP_KEY );
    setVimbaFeature( vimbaSystem, "ActionGroupMask", ACTION_GROUP_MASK );

    vimbaRunCommand( vimbaSystem, "ActionCommand" );
}

// ImageDisparityWidget
ImageDisparityWidget::ImageDisparityWidget( QWidget* parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void ImageDisparityWidget::initialize()
{
    m_disparityWidget = new DisparityWidgetBase( this );
    m_iconsWidget = new DisparityIconsWidget( this );

    addWidget( m_disparityWidget );
    addWidget( m_iconsWidget );

    connect( m_iconsWidget, SIGNAL( iconActivated( DisparityIcon* ) ), this, SLOT( updateFrame( DisparityIcon* ) ) );

    connect( m_disparityWidget, &DisparityWidgetBase::valueChanged, this, static_cast< void ( ImageDisparityWidget::* )() >( &ImageDisparityWidget::updateFrame ) );

}

BMControlWidget *ImageDisparityWidget::bmControlWidget() const
{
    return m_disparityWidget->bmControlWidget();
}

GMControlWidget *ImageDisparityWidget::gmControlWidget() const
{
    return m_disparityWidget->gmControlWidget();
}

bool ImageDisparityWidget::loadCalibrationFile( const QString &fileName )
{
    m_disparityWidget->loadCalibrationFile( fileName );
}

void ImageDisparityWidget::addIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    m_iconsWidget->addIcon( new DisparityIcon( leftImage, rightImage, 0 ) );
}

void ImageDisparityWidget::insertIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    m_iconsWidget->insertIcon( new DisparityIcon( leftImage, rightImage, 0 ) );
}

void ImageDisparityWidget::loadIcon( const QString &leftFileName, const QString &rightFileName )
{
    CvImage leftImg = cv::imread( leftFileName.toStdString() );
    CvImage rightImg = cv::imread( rightFileName.toStdString() );

    if ( !leftImg.empty() && !rightImg.empty() ) {
        addIcon( leftImg, rightImg );

    }

}

void ImageDisparityWidget::loadCalibrationDialog()
{
    m_disparityWidget->loadCalibrationDialog();
}

void ImageDisparityWidget::importDialog()
{
    StereoFilesListDialog dlg( this );

    if ( dlg.exec() == StereoFilesListDialog::Accepted ) {
        auto leftFileNames = dlg.leftFileNames();
        auto rightFileNames = dlg.rightFileNames();

        for ( auto i = 0; i < leftFileNames.size(); ++i ) {
            loadIcon( leftFileNames[i], rightFileNames[i] );
        }

    }


}

void ImageDisparityWidget::updateFrame()
{
    auto icon = m_iconsWidget->currentIcon();

    if ( icon )
        updateFrame( icon );

}

void ImageDisparityWidget::updateFrame( DisparityIcon* icon )
{
    m_disparityWidget->updateFrame( icon->leftImage(), icon->rightImage() );
}

