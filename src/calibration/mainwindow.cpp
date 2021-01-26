#include "src/common/precompiled.h"

#include "mainwindow.h"

#include "calibrationwidget.h"
#include "documentwidget.h"

#include "src/common/ipwidget.h"

#include "calibrationchoicedialog.h"

MainWindow::MainWindow( QWidget *parent )
    : DocumentMainWindow( parent )
{
    initialize();
}

MainWindow::MainWindow( const QString &cameraIp, QWidget *parent )
    : DocumentMainWindow( parent )
{
    initialize( cameraIp );
}

MainWindow::MainWindow( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent )
    : DocumentMainWindow( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void MainWindow::initialize( const QString &cameraIp )
{
    initialize();

    addMonocularCameraCalibrationDocument( cameraIp );
}

void MainWindow::initialize( const QString &leftCameraIp, const QString &rightCameraIp )
{
    initialize();

    addStereoCameraCalibrationDocument( leftCameraIp, rightCameraIp );

}

void MainWindow::initialize()
{
    setupActions();
    setupMenus();
    setupToolBars();
    setupStatusBar();

    startTimer( m_grabInterval );
    setAttribute( Qt::WA_DeleteOnClose );

}

QPointer< MonocularCameraCalibrationDocument > MainWindow::addMonocularCameraCalibrationDocument(const QString &cameraIp )
{
    QPointer< MonocularCameraCalibrationDocument > ret = new MonocularCameraCalibrationDocument( cameraIp, this );

    addDocument( ret );

    return ret;
}

QPointer< StereoCameraCalibrationDocument > MainWindow::addStereoCameraCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp )
{
    QPointer< StereoCameraCalibrationDocument > ret = new StereoCameraCalibrationDocument( leftCameraIp, rightCameraIp, this );

    addDocument( ret );

    return ret;
}

QPointer< MonocularImageCalibrationDocument > MainWindow::addMonocularCalibrationDocument()
{

    QPointer< MonocularImageCalibrationDocument > ret = new MonocularImageCalibrationDocument( this );

    addDocument( ret );

    return ret;
}

QPointer< StereoImageCalibrationDocument > MainWindow::addStereoCalibrationDocument()
{
    QPointer< StereoImageCalibrationDocument > ret = new StereoImageCalibrationDocument( this );

    addDocument( ret );

    return ret;
}

QPointer< MonocularReportDocument > MainWindow::addMonocularReportDocument()
{
    QPointer< MonocularReportDocument > ret = new MonocularReportDocument( this );

    addDocument( ret );

    return ret;
}

QPointer< StereoReportDocument > MainWindow::addStereoReportDocument()
{
    QPointer< StereoReportDocument > ret = new StereoReportDocument( this );

    addDocument( ret );

    return ret;
}

CalibrationDocumentBase *MainWindow::currentCalibrationDocument() const
{
    return getCurrentDocument< CalibrationDocumentBase >();
}

CameraCalibrationDocumentBase *MainWindow::currentCameraCalibrationDocument() const
{
    return getCurrentDocument< CameraCalibrationDocumentBase >();
}

MonocularCameraCalibrationDocument *MainWindow::currentMonocularCalibrationDocument() const
{
    return getCurrentDocument< MonocularCameraCalibrationDocument >();
}

StereoCameraCalibrationDocument *MainWindow::currentStereoCalibrationDocument() const
{
    return getCurrentDocument< StereoCameraCalibrationDocument >();
}

TrippleCalibrationDocument *MainWindow::currentTrippleCalibrationDocument() const
{
    return getCurrentDocument< TrippleCalibrationDocument >();
}

ReportDocumentBase *MainWindow::currentReportDocument() const
{
    return getCurrentDocument< ReportDocumentBase >();
}

void MainWindow::setupActions()
{
    m_newCalibrationDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New calibration document" ), this );
    m_openAction = new QAction( QIcon( ":/resources/images/open.ico" ), tr( "Open" ), this );
    m_saveAction = new QAction( QIcon( ":/resources/images/save.ico" ), tr( "Save" ), this );

    m_importAction = new QAction( QIcon( ":/resources/images/import.ico" ), tr( "Import" ), this );
    m_exportAction = new QAction( QIcon( ":/resources/images/export.ico" ), tr( "Export" ), this );

    m_grabAction = new QAction( QIcon( ":/resources/images/grab.ico" ), tr( "Grab" ), this );

    m_autoGrabAction = new QAction( QIcon( ":/resources/images/camera.ico" ), tr( "Autograb" ), this );
    m_autoGrabAction->setCheckable( true );
    m_autoGrabAction->setChecked( true );

    m_exportYamlAction = new QAction( QIcon( ":/resources/images/xml.ico" ), tr( "Export YAML" ), this );
    m_calculateAction = new QAction( QIcon( ":/resources/images/checkerflag.ico" ), tr( "Calculate" ), this );

    m_clearIconsAction = new QAction( QIcon( ":/resources/images/trash.ico" ), tr( "Clear" ), this );

    m_settingsAction = new QAction( QIcon( ":/resources/images/settings.ico" ), tr( "Settings" ), this );
    m_exitAction = new QAction( QIcon( ":/resources/images/power.ico" ), tr( "Exit" ), this );
    m_aboutAction = new QAction( QIcon( ":/resources/images/help.ico" ), tr( "About" ), this );

    connect( m_newCalibrationDocumentAction, &QAction::triggered, this, &MainWindow::choiceCalibrationDialog );

    connect( m_importAction, &QAction::triggered, this, &MainWindow::importDialog );
    connect( m_exportAction, &QAction::triggered, this, &MainWindow::exportDialog );

    connect( m_grabAction, &QAction::triggered, this, &MainWindow::grabFrame );
    connect( m_exportYamlAction, &QAction::triggered, this, &MainWindow::exportYamlResults );
    connect( m_calculateAction, &QAction::triggered, this, &MainWindow::calculate );
    connect( m_clearIconsAction, &QAction::triggered, this, &MainWindow::clearIcons );
    connect( m_settingsAction, &QAction::triggered, this, &MainWindow::settingsDialog );
    connect( m_exitAction, &QAction::triggered, this, &MainWindow::close );
}

void MainWindow::setupMenus()
{
    m_menuBar = new QMenuBar(this);

    auto fileMenu = m_menuBar->addMenu( tr( "File" ) );
    fileMenu->addAction( m_newCalibrationDocumentAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_openAction );
    fileMenu->addAction( m_saveAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_importAction );
    fileMenu->addAction( m_exportAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_exitAction );

    auto actionsMenu = m_menuBar->addMenu( tr( "Actions" ) );
    actionsMenu->addAction( m_grabAction );
    actionsMenu->addAction( m_autoGrabAction );
    actionsMenu->addSeparator();
    actionsMenu->addAction( m_exportYamlAction );
    actionsMenu->addSeparator();
    actionsMenu->addAction( m_calculateAction );
    actionsMenu->addSeparator();
    actionsMenu->addAction( m_clearIconsAction );
    actionsMenu->addSeparator();
    actionsMenu->addAction( m_settingsAction );

    auto helpMenu = m_menuBar->addMenu( tr( "Help" ) );
    helpMenu->addAction( m_aboutAction );

    setMenuBar(m_menuBar);

}

void MainWindow::setupToolBars()
{
    // Настройки панели инструментов проекта
    m_toolBar = new QToolBar( tr( "Project tool bar" ), this );
    addToolBar( m_toolBar );

    m_toolBar->addAction( m_newCalibrationDocumentAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_openAction );
    m_toolBar->addAction( m_saveAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_importAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_grabAction );
    m_toolBar->addAction( m_autoGrabAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_exportYamlAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_calculateAction );
}

void MainWindow::timerEvent( QTimerEvent * )
{
    if (m_autoGrabAction->isChecked()) {
        grabFrame();
    }

}

void MainWindow::importDialog()
{
    auto doc = currentCalibrationDocument();

    if (doc)
        doc->importDialog();

}

void MainWindow::exportDialog()
{
    auto doc = currentCalibrationDocument();

    if (doc)
        doc->exportDialog();

}

void MainWindow::grabFrame()
{
    auto doc = currentCameraCalibrationDocument();

    if (doc)
        doc->grabFrame();
}

void MainWindow::exportYamlResults()
{
    auto doc = currentReportDocument();

    if (doc)
        doc->exportYaml();

}

void MainWindow::calculate()
{
    auto doc = currentCalibrationDocument();

    if (doc)
        doc->calculate();
}

void MainWindow::settingsDialog()
{
}

void MainWindow::clearIcons()
{
    auto doc = currentCalibrationDocument();

    if (doc)
        doc->clearIcons();
}

void MainWindow::choiceCalibrationDialog()
{
     CalibrationChoiceDialog dlg( this );

     if ( dlg.exec() == DialogBase::Accepted ) {
         switch ( dlg.selectedType() ) {
         case CalibrationChoiceDialog::MONOCULAR_IMAGE:
             addMonocularImageCalibrationDialog();
             break;
         case CalibrationChoiceDialog::STEREO_IMAGE:
             addStereoImageCalibrationDialog();
             break;
         case CalibrationChoiceDialog::MONOCULAR_CAMERA:
             addMonocularCameraCalibrationDialog();
             break;
         case CalibrationChoiceDialog::STEREO_CAMERA:
             addStereoCameraCalibrationDialog();
             break;
         default:
             break;
         }
     }

}

void MainWindow::addMonocularImageCalibrationDialog()
{
    addMonocularCalibrationDocument();
}

void MainWindow::addStereoImageCalibrationDialog()
{
    addStereoCalibrationDocument();
}

void MainWindow::addMonocularCameraCalibrationDialog()
{
    CameraIPDialog dialog( this );

    if ( dialog.exec() == DialogBase::Accepted )
        addMonocularCameraCalibrationDocument( dialog.ip() );
}

void MainWindow::addStereoCameraCalibrationDialog()
{
    StereoIPDialog dialog( this );

    if ( dialog.exec() == DialogBase::Accepted )
        addStereoCameraCalibrationDocument( dialog.leftIp(), dialog.rightIp() );

}
