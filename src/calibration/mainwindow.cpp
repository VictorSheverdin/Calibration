#include "src/common/precompiled.h"

#include "mainwindow.h"

#include "calibrationwidget.h"
#include "documentarea.h"

#include "src/common/ipwidget.h"

MainWindow::MainWindow( QWidget *parent )
    : QMainWindow(parent)
{
    initialize();
}

MainWindow::MainWindow( const QString &cameraIp, QWidget *parent )
    : QMainWindow(parent)
{
    initialize( cameraIp );
}

MainWindow::MainWindow( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent )
    : QMainWindow(parent)
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
    setupDocuments();
    setupActions();
    setupMenus();
    setupToolBars();
    setupStatusBar();

    startTimer( m_grabInterval );
    setAttribute( Qt::WA_DeleteOnClose );

}

void MainWindow::addDocument( DocumentBase *document )
{
    m_documentArea->addDocument( document );
}

void MainWindow::addMonocularCameraCalibrationDocument(const QString &cameraIp )
{
    addDocument( new MonocularCameraCalibrationDocument( cameraIp, this ) );
}

void MainWindow::addStereoCameraCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp )
{
    addDocument( new StereoCameraCalibrationDocument( leftCameraIp, rightCameraIp, this ) );
}

void MainWindow::addMonocularCalibrationDocument()
{
    addDocument( new MonocularCalibrationDocument( this ) );
}

void MainWindow::addStereoCalibrationDocument()
{
    addDocument( new StereoCalibrationDocument( this ) );
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

ReportDocument *MainWindow::currentReportDocument() const
{
    return getCurrentDocument< ReportDocument >();
}

void MainWindow::setupDocuments()
{
    m_documentArea = new DocumentArea( this );

    setCentralWidget( m_documentArea );
}

void MainWindow::setupActions()
{
    m_newMonocularDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New monocular calibration" ), this );
    m_newStereoDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New stereo calibration" ), this );
    m_newMonocularCameraDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New monocular camera calibration" ), this );
    m_newStereoCameraDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New stereo camera calibration" ), this );
    m_openAction = new QAction( QIcon( ":/resources/images/open.ico" ), tr( "Open" ), this );
    m_saveAction = new QAction( QIcon( ":/resources/images/save.ico" ), tr( "Save" ), this );

    m_importAction = new QAction( QIcon( ":/resources/images/export.ico" ), tr( "Import" ), this );
    m_exportAction = new QAction( QIcon( ":/resources/images/import.ico" ), tr( "Export" ), this );

    m_grabAction = new QAction( QIcon( ":/resources/images/grab.ico" ), tr( "Grab" ), this );

    m_autoGrabAction = new QAction( QIcon( ":/resources/images/camera.ico" ), tr( "Autograb" ), this );
    m_autoGrabAction->setCheckable( true );
    m_autoGrabAction->setChecked( true );

    m_calculateAction = new QAction( QIcon( ":/resources/images/checkerflag.ico" ), tr( "Calculate" ), this );

    m_clearIconsAction = new QAction( QIcon( ":/resources/images/trash.ico" ), tr( "Clear" ), this );

    m_settingsAction = new QAction( QIcon( ":/resources/images/settings.ico" ), tr( "Settings" ), this );
    m_exitAction = new QAction( QIcon( ":/resources/images/power.ico" ), tr( "Exit" ), this );
    m_aboutAction = new QAction( QIcon( ":/resources/images/help.ico" ), tr( "About" ), this );

    connect( m_newMonocularDocumentAction, &QAction::triggered, this, &MainWindow::addMonocularCalibrationDialog );
    connect( m_newStereoDocumentAction, &QAction::triggered, this, &MainWindow::addStereoCalibrationDialog );

    connect( m_newMonocularCameraDocumentAction, &QAction::triggered, this, &MainWindow::addMonocularCameraCalibrationDialog );
    connect( m_newStereoCameraDocumentAction, &QAction::triggered, this, &MainWindow::addStereoCameraCalibrationDialog );

    connect( m_importAction, &QAction::triggered, this, &MainWindow::importDialog );
    connect( m_exportAction, &QAction::triggered, this, &MainWindow::exportDialog );

    connect( m_grabAction, &QAction::triggered, this, &MainWindow::grabFrame );
    connect( m_calculateAction, &QAction::triggered, this, &MainWindow::calculate );
    connect( m_clearIconsAction, &QAction::triggered, this, &MainWindow::clearIcons );
    connect( m_settingsAction, &QAction::triggered, this, &MainWindow::settingsDialog );
    connect( m_exitAction, &QAction::triggered, this, &MainWindow::close );

}

void MainWindow::setupMenus()
{
    m_menuBar = new QMenuBar(this);

    auto fileMenu = m_menuBar->addMenu( tr( "File" ) );
    fileMenu->addAction( m_newMonocularDocumentAction );
    fileMenu->addAction( m_newStereoDocumentAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_newMonocularCameraDocumentAction );
    fileMenu->addAction( m_newStereoCameraDocumentAction );
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

    m_toolBar->addAction( m_newMonocularDocumentAction );
    m_toolBar->addAction( m_newStereoDocumentAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_newMonocularCameraDocumentAction );
    m_toolBar->addAction( m_newStereoCameraDocumentAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_openAction );
    m_toolBar->addAction( m_saveAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_grabAction );
    m_toolBar->addAction( m_autoGrabAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_calculateAction );
}

void MainWindow::setupStatusBar()
{
    m_statusBar = new QStatusBar( this );
    setStatusBar( m_statusBar );
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

void MainWindow::addMonocularCalibrationDialog()
{
    addMonocularCalibrationDocument();
}

void MainWindow::addStereoCalibrationDialog()
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
