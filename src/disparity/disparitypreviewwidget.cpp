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
    setAttribute( Qt::WA_DeleteOnClose );

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

    m_bmProcessor = std::shared_ptr< BMDisparityProcessor >( new BMDisparityProcessor );
    m_gmProcessor = std::shared_ptr< GMDisparityProcessor >( new GMDisparityProcessor );
    m_bmGpuProcessor = std::shared_ptr< BMGPUDisparityProcessor >( new BMGPUDisparityProcessor );
    m_bpProcessor = std::shared_ptr< BPDisparityProcessor >( new BPDisparityProcessor );
    m_csbpProcessor = std::shared_ptr< CSBPDisparityProcessor >( new CSBPDisparityProcessor );
    m_elasProcessor = std::shared_ptr< ElasDisparityProcessor >( new ElasDisparityProcessor );

    m_processor = std::shared_ptr<StereoResultProcessor>( new StereoResultProcessor );

    m_processorThread.setProcessor( m_processor );

    connect( &m_processorThread, &ProcessorThread::frameProcessed, this, &DisparityWidgetBase::updateFrame );

}

BMControlWidget *DisparityWidgetBase::bmControlWidget() const
{
    return m_controlWidget->bmControlWidget();
}

GMControlWidget *DisparityWidgetBase::gmControlWidget() const
{
    return m_controlWidget->gmControlWidget();
}

BMGPUControlWidget *DisparityWidgetBase::bmGpuControlWidget() const
{
    return m_controlWidget->bmGpuControlWidget();
}

BPControlWidget *DisparityWidgetBase::bpControlWidget() const
{
    return m_controlWidget->bpControlWidget();
}

void DisparityWidgetBase::loadCalibrationFile( const QString &fileName )
{
    m_processor->loadYaml( fileName.toStdString() );
}

void DisparityWidgetBase::loadCalibrationDialog()
{
    auto file = QFileDialog::getOpenFileName(
                        this,
                        tr( "Select calibration file" ),
                        QString(),
                        "Calibration files (*.yaml)"
                );

    if ( !file.isEmpty() )
        loadCalibrationFile( file );
}

void DisparityWidgetBase::processFrame( const StereoFrame &frame )
{
     if ( !frame.empty() ) {

        if ( m_controlWidget->isBmMethod() ) {
            m_bmProcessor->setBlockSize( bmControlWidget()->sadWindowSize() );
            m_bmProcessor->setMinDisparity( bmControlWidget()->minDisparity() );
            m_bmProcessor->setNumDisparities( bmControlWidget()->numDisparities() );
            m_bmProcessor->setPreFilterSize( bmControlWidget()->prefilterSize() );
            m_bmProcessor->setPreFilterCap( bmControlWidget()->prefilterCap() );
            m_bmProcessor->setTextureThreshold( bmControlWidget()->textureThreshold() );
            m_bmProcessor->setUniquenessRatio( bmControlWidget()->uniquessRatio() );
            m_bmProcessor->setSpeckleWindowSize( bmControlWidget()->speckleWindowSize() );
            m_bmProcessor->setSpeckleRange( bmControlWidget()->speckleRange() );
            m_bmProcessor->setDisp12MaxDiff( bmControlWidget()->disp12MaxDiff() );

            m_processor->setDisparityProcessor( m_bmProcessor );

        }
        else if ( m_controlWidget->isBmGpuMethod() ) {
            m_bmGpuProcessor->setBlockSize( bmGpuControlWidget()->sadWindowSize() );
            m_bmGpuProcessor->setNumDisparities( bmGpuControlWidget()->numDisparities() );
            m_bmGpuProcessor->setPreFilterCap( bmGpuControlWidget()->prefilterCap() );
            m_bmGpuProcessor->setTextureThreshold( bmGpuControlWidget()->textureThreshold() );

            m_processor->setDisparityProcessor( m_bmGpuProcessor );

        }
        else if ( m_controlWidget->isGmMethod() ) {
            m_gmProcessor->setMode( gmControlWidget()->mode() );
            m_gmProcessor->setPreFilterCap( gmControlWidget()->prefilterCap() );
            m_gmProcessor->setBlockSize( gmControlWidget()->sadWindowSize() );
            m_gmProcessor->setMinDisparity( gmControlWidget()->minDisparity() );
            m_gmProcessor->setNumDisparities( gmControlWidget()->numDisparities() );
            m_gmProcessor->setUniquenessRatio( gmControlWidget()->uniquessRatio() );
            m_gmProcessor->setSpeckleWindowSize( gmControlWidget()->speckleWindowSize() );
            m_gmProcessor->setSpeckleRange( gmControlWidget()->speckleRange() );
            m_gmProcessor->setDisp12MaxDiff( gmControlWidget()->disp12MaxDiff() );
            m_gmProcessor->setP1( gmControlWidget()->p1() );
            m_gmProcessor->setP2( gmControlWidget()->p2() );

            m_processor->setDisparityProcessor( m_gmProcessor );

        }
        else if ( m_controlWidget->isBpMethod() ) {
            m_bpProcessor->setNumDisparities( bpControlWidget()->numDisparities() );
            m_bpProcessor->setNumIterations( bpControlWidget()->numIterations() );
            m_bpProcessor->setNumLevels( bpControlWidget()->numLevels() );
            m_bpProcessor->setMaxDataTerm( bpControlWidget()->maxDataTerm() );
            m_bpProcessor->setDataWeight( bpControlWidget()->dataWeight() );
            m_bpProcessor->setMaxDiscTerm( bpControlWidget()->maxDiscTerm() );
            m_bpProcessor->setDiscSingleJump( bpControlWidget()->discSingleJump() );

            m_processor->setDisparityProcessor( m_bpProcessor );

        }
        else if ( m_controlWidget->isCsbpMethod() ) {
            m_processor->setDisparityProcessor( m_csbpProcessor );

        }
        else if ( m_controlWidget->isElasMethod() ) {
            m_processor->setDisparityProcessor( m_elasProcessor );

        }

        m_processorThread.process( frame );

    }

}

void DisparityWidgetBase::updateFrame()
{
    auto result = m_processorThread.result();

    m_view->rectifyView()->setImage( result.previewImage() );

    m_view->disparityView()->setImage( result.colorizedDisparity() );

    if ( result.pointCloud() && !result.pointCloud()->empty() ) {
        m_3dWidget->setPointCloud( result.pointCloud() );
        m_3dWidget->update();

    }


}

// CameraDisparityWidget
CameraDisparityWidget::CameraDisparityWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : DisparityWidgetBase( parent ), m_camera( leftCameraIp.toStdString(), rightCameraIp.toStdString(), parent )
{
    initialize();
}

void CameraDisparityWidget::initialize()
{
    connect( &m_camera, &StereoCamera::receivedFrame, this, &CameraDisparityWidget::updateFrame );
}

void CameraDisparityWidget::updateFrame()
{
    if ( m_updateMutex.tryLock() ) {

        auto frame = m_camera.getFrame();

        DisparityWidgetBase::processFrame( frame );

        m_updateMutex.unlock();

    }

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

    dropIconCount();
}

void ImageDisparityWidget::dropIconCount()
{
    m_iconCount = 0;
}

BMControlWidget *ImageDisparityWidget::bmControlWidget() const
{
    return m_disparityWidget->bmControlWidget();
}

GMControlWidget *ImageDisparityWidget::gmControlWidget() const
{
    return m_disparityWidget->gmControlWidget();
}

void ImageDisparityWidget::loadCalibrationFile( const QString &fileName )
{
    m_disparityWidget->loadCalibrationFile( fileName );
}

void ImageDisparityWidget::addIcon( const QString &leftFileName, const QString &rightFileName )
{
    CvImage leftImg = cv::imread( leftFileName.toStdString() );
    CvImage rightImg = cv::imread( rightFileName.toStdString() );

    if ( !leftImg.empty() && !rightImg.empty() ) {
        m_iconsWidget->addIcon( new DisparityIcon( makeOverlappedPreview( leftImg, rightImg ) , leftFileName, rightFileName, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) ) );
    }

}

void ImageDisparityWidget::loadCalibrationDialog()
{
    m_disparityWidget->loadCalibrationDialog();

    updateFrame();
}

void ImageDisparityWidget::importDialog()
{
    StereoFilesListDialog dlg( this );

    if ( dlg.exec() == StereoFilesListDialog::Accepted ) {
        auto leftFileNames = dlg.leftFileNames();
        auto rightFileNames = dlg.rightFileNames();

        for ( auto i = 0; i < leftFileNames.size(); ++i ) {
            addIcon( leftFileNames[i], rightFileNames[i] );
        }

    }

    updateFrame();

}

void ImageDisparityWidget::clearIcons()
{
    m_iconsWidget->clear();

    dropIconCount();
}

void ImageDisparityWidget::updateFrame()
{
    auto icon = m_iconsWidget->currentIcon();

    if ( icon )
        updateFrame( icon );

}

void ImageDisparityWidget::updateFrame( DisparityIcon* icon )
{
    m_disparityWidget->processFrame( icon->stereoFrame() );
}

