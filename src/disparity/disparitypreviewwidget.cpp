#include "src/common/precompiled.h"

#include "disparitypreviewwidget.h"

#include "src/common/imagewidget.h"
#include "disparitycontrolwidget.h"
#include "disparityiconswidget.h"

#include <opencv2/ximgproc.hpp>

#include "src/common/pclwidget.h"

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

// ReconstructionViewWidget
ReconstructionViewWidget::ReconstructionViewWidget( QWidget* parent )
    : PCLWidget( parent )
{
    initialize();
}

void ReconstructionViewWidget::initialize()
{
    m_pclViewer->setCameraPosition( 0, 0, -10, 0, -1, 0 );
    m_pclViewer->setCameraClipDistances( 0.1, 10000 );

    m_pclViewer->registerPointPickingCallback( ReconstructionViewWidget::pickingEventHandler, m_pclViewer.get() );

}

void ReconstructionViewWidget::pickingEventHandler( const pcl::visualization::PointPickingEvent &event, void *viewer_void )
{
    float x, y, z;

    if ( event.getPointIndex() == -1 ) {
        return;
    }

    event.getPoint( x, y, z );

    auto distance = sqrt( x * x + y * y + z * z );

    auto distanceString = QString::number( distance );

    application()->setStatusBarText( tr( "x = %1, y = %2, z = %3, distance = %4" )
                                     .arg( QString::number( x ) ).arg( QString::number( y ) ).arg( QString::number( z ) ).arg( distanceString ) );

    auto widget = reinterpret_cast< pcl::visualization::PCLVisualizer * >( viewer_void );

    widget->removeText3D( "Distance" );
    widget->addText3D( distanceString.toStdString(), pcl::PointXYZ( x, y, z ), 1.0, 1.0, 0, 0, "Distance" );

}

// ControlDisparityWidget
ControlDisparityWidget::ControlDisparityWidget( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void ControlDisparityWidget::initialize()
{
    setAttribute( Qt::WA_DeleteOnClose );

    auto tabWidget = new QTabWidget( this );

    m_view = new DisparityPreviewWidget( this );
    tabWidget->resize( 1200, 800 );
    m_3dWidget = new ReconstructionViewWidget( this );

    tabWidget->addTab( m_view, tr( "Disparity" ) );
    tabWidget->addTab( m_3dWidget, tr( "3D priview" ) );

    m_controlWidget = new DisparityControlWidget( this );

    addWidget( tabWidget );
    addWidget( m_controlWidget );

    connect( m_controlWidget, &DisparityControlWidget::valueChanged, this, &ControlDisparityWidget::valueChanged );

    m_bmProcessor = std::shared_ptr< BMDisparityProcessor >( new BMDisparityProcessor );
    m_gmProcessor = std::shared_ptr< GMDisparityProcessor >( new GMDisparityProcessor );
    m_bmGpuProcessor = std::shared_ptr< BMGPUDisparityProcessor >( new BMGPUDisparityProcessor );
    m_bpProcessor = std::shared_ptr< BPDisparityProcessor >( new BPDisparityProcessor );
    m_csbpProcessor = std::shared_ptr< CSBPDisparityProcessor >( new CSBPDisparityProcessor );
    m_elasProcessor = std::shared_ptr< ElasDisparityProcessor >( new ElasDisparityProcessor );

    m_processor = std::shared_ptr< StereoResultProcessor >( new StereoResultProcessor );

    m_processorThread.setProcessor( m_processor );

    connect( &m_processorThread, &ProcessorThread::frameProcessed, this, &ControlDisparityWidget::updateFrame );

}

BMControlWidget *ControlDisparityWidget::bmControlWidget() const
{
    return m_controlWidget->bmControlWidget();
}

GMControlWidget *ControlDisparityWidget::gmControlWidget() const
{
    return m_controlWidget->gmControlWidget();
}

BMGPUControlWidget *ControlDisparityWidget::bmGpuControlWidget() const
{
    return m_controlWidget->bmGpuControlWidget();
}

BPControlWidget *ControlDisparityWidget::bpControlWidget() const
{
    return m_controlWidget->bpControlWidget();
}

void ControlDisparityWidget::loadCalibrationFile( const QString &fileName )
{
    m_processor->loadYaml( fileName.toStdString() );
}

void ControlDisparityWidget::loadCalibrationDialog()
{
    auto file = QFileDialog::getOpenFileName(
                        this,
                        tr( "Select calibration file" ),
                        QString(),
                        tr( "Calibration files (*.yaml)" )
                );

    if ( !file.isEmpty() )
        loadCalibrationFile( file );
}

void ControlDisparityWidget::processFrame( const StampedStereoImage &frame )
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

void ControlDisparityWidget::processDisparity( const CvImage &color, const CvImage &disparity )
{
    auto cloud = m_processor->process( color, disparity );

    if ( cloud ) {
        m_3dWidget->setPointCloud( cloud );
        m_3dWidget->update();

    }

}

void ControlDisparityWidget::updateFrame()
{
    auto result = m_processorThread.result();

    m_view->rectifyView()->setImage( result.previewImage() );

    m_view->disparityView()->setImage( result.colorizedDisparity() );

    if ( result.pointCloud() && !result.pointCloud()->empty() ) {
        m_3dWidget->setPointCloud( result.pointCloud() );
        m_3dWidget->update();

    }

}

// ViewDisparityWidget
ViewDisparityWidget::ViewDisparityWidget( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void ViewDisparityWidget::initialize()
{
    setAttribute( Qt::WA_DeleteOnClose );

    m_view = new DisparityPreviewWidget( this );
    m_3dWidget = new ReconstructionViewWidget( this );

    addWidget( m_view );
    addWidget( m_3dWidget );

    m_processor = std::shared_ptr< StereoResultProcessor >( new StereoResultProcessor );
}

void ViewDisparityWidget::loadCalibrationFile( const QString &fileName )
{
    m_processor->loadYaml( fileName.toStdString() );
}

void ViewDisparityWidget::loadCalibrationDialog()
{
    auto file = QFileDialog::getOpenFileName(
                        this,
                        tr( "Select calibration file" ),
                        QString(),
                        tr( "Calibration files (*.yaml)" )
                );

    if ( !file.isEmpty() )
        loadCalibrationFile( file );
}

void ViewDisparityWidget::processDisparity( const CvImage &color, const CvImage &disparity )
{
    auto cloud = m_processor->process( color, disparity );

    m_view->rectifyView()->setImage( color );

    m_view->disparityView()->setImage( colorizeDisparity( disparity ) );

    if ( cloud ) {
        m_3dWidget->setPointCloud( cloud );
        m_3dWidget->update();

    }

}

// CameraDisparityWidget
CameraDisparityWidget::CameraDisparityWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : ControlDisparityWidget( parent ), m_camera( leftCameraIp.toStdString(), rightCameraIp.toStdString(), parent )
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

        ControlDisparityWidget::processFrame( frame );

        m_updateMutex.unlock();

    }

}

// DiskDisparityWidget
DiskDisparityWidget::DiskDisparityWidget( QWidget* parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void DiskDisparityWidget::initialize()
{
    m_iconsWidget = new DisparityIconsWidget( this );

    dropIconCount();
}

void DiskDisparityWidget::dropIconCount()
{
    m_iconCount = 0;
}

void DiskDisparityWidget::clearIcons()
{
    m_iconsWidget->clear();

    dropIconCount();
}

// StereoDisparityWidget
StereoDisparityWidget::StereoDisparityWidget( QWidget* parent )
    : DiskDisparityWidget( parent )
{
    initialize();
}

void StereoDisparityWidget::initialize()
{
    m_disparityWidget = new ControlDisparityWidget( this );

    addWidget( m_disparityWidget );
    addWidget( m_iconsWidget );

    connect( m_iconsWidget, SIGNAL( iconActivated( DisparityIconBase* ) ), this, SLOT( updateFrame() ) );

    connect( m_disparityWidget, &ControlDisparityWidget::valueChanged, this, static_cast< void ( StereoDisparityWidget::* )() >( &StereoDisparityWidget::updateFrame ) );
}


BMControlWidget *StereoDisparityWidget::bmControlWidget() const
{
    return m_disparityWidget->bmControlWidget();
}

GMControlWidget *StereoDisparityWidget::gmControlWidget() const
{
    return m_disparityWidget->gmControlWidget();
}

void StereoDisparityWidget::loadCalibrationFile( const QString &fileName )
{
    m_disparityWidget->loadCalibrationFile( fileName );
}

void StereoDisparityWidget::addStereoIcon( const QString &leftFileName, const QString &rightFileName )
{
    CvImage leftImg = cv::imread( leftFileName.toStdString() );
    CvImage rightImg = cv::imread( rightFileName.toStdString() );

    if ( !leftImg.empty() && !rightImg.empty() )
        m_iconsWidget->addIcon( new DisparityIcon( makeOverlappedPreview( leftImg, rightImg ) , leftFileName, rightFileName, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) ) );

}

void StereoDisparityWidget::loadCalibrationDialog()
{
    m_disparityWidget->loadCalibrationDialog();

    updateFrame();
}

void StereoDisparityWidget::importStereoDialog()
{
    StereoFilesListDialog dlg( this );

    if ( dlg.exec() == StereoFilesListDialog::Accepted ) {
        auto leftFileNames = dlg.leftFileNames();
        auto rightFileNames = dlg.rightFileNames();

        for ( auto i = 0; i < leftFileNames.size(); ++i )
            addStereoIcon( leftFileNames[i], rightFileNames[i] );


    }

    updateFrame();

}

void StereoDisparityWidget::updateFrame()
{
    auto icon = m_iconsWidget->currentIcon();

    auto disparityIcon = dynamic_cast< DisparityIcon * >( icon );

    if ( disparityIcon )
        updateFrame( disparityIcon );

}

void StereoDisparityWidget::updateFrame( DisparityIcon* icon )
{
    m_disparityWidget->processFrame( icon->stereoFrame() );
}

// FileDisparityWidget
FileDisparityWidget::FileDisparityWidget( QWidget* parent )
    : DiskDisparityWidget( parent )
{
    initialize();
}

void FileDisparityWidget::initialize()
{
    m_disparityWidget = new ViewDisparityWidget( this );

    addWidget( m_disparityWidget );
    addWidget( m_iconsWidget );

    connect( m_iconsWidget, SIGNAL( iconActivated( DisparityIconBase* ) ), this, SLOT( updateFrame() ) );

}

void FileDisparityWidget::loadCalibrationFile( const QString &fileName )
{
    m_disparityWidget->loadCalibrationFile( fileName );
}

void FileDisparityWidget::addDisparityIcon( const QString &colorFileName , const QString &disparityFileName)
{
    CvImage img = cv::imread( colorFileName.toStdString() );

    if ( !img.empty() )
        m_iconsWidget->addIcon( new DisparityResultIcon( img, colorFileName, disparityFileName, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) ) );

}

void FileDisparityWidget::loadCalibrationDialog()
{
    m_disparityWidget->loadCalibrationDialog();
}

void FileDisparityWidget::importDisparityDialog()
{
    StereoFilesListDialog dlg( this );

    if ( dlg.exec() == StereoFilesListDialog::Accepted ) {
        auto leftFileNames = dlg.leftFileNames();
        auto rightFileNames = dlg.rightFileNames();

        for ( auto i = 0; i < leftFileNames.size(); ++i )
            addDisparityIcon( leftFileNames[i], rightFileNames[i] );

    }

    updateFrame();

}

void FileDisparityWidget::updateFrame()
{
    auto icon = m_iconsWidget->currentIcon();

    auto disparityIcon = dynamic_cast< DisparityResultIcon * >( icon );

    if ( disparityIcon )
        updateFrame( disparityIcon );

}

void FileDisparityWidget::updateFrame( DisparityResultIcon* icon )
{
    m_disparityWidget->processDisparity( icon->colorImage(), icon->disparityImage() );
}

