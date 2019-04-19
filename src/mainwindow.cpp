#include "precompiled.h"

#include "mainwindow.h"

#include "calibrationwidget.h"

MainWindow::MainWindow( const std::string &cameraIp, QWidget *parent)
    : QMainWindow(parent)
{
    initialize( cameraIp );
}

MainWindow::MainWindow( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent )
    : QMainWindow(parent)
{
    initialize( leftCameraIp, rightCameraIp );
}

void MainWindow::initialize( const std::string &cameraIp )
{
    m_widget = new MonocularCalibrationWidget( cameraIp, this );

    initialize();
}

void MainWindow::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    m_widget = new StereoCalibrationWidget( leftCameraIp, rightCameraIp, this );

    initialize();
}

void MainWindow::initialize()
{
    setCentralWidget( m_widget );

    setupActions();
    setupMenus();
    setupToolBars();
    setupStatusBar();

    startTimer( m_grabInterval );
    setAttribute( Qt::WA_DeleteOnClose );

}

void MainWindow::setupActions()
{
    m_newAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New" ), this );
    m_openAction = new QAction( QIcon( ":/resources/images/open.ico" ), tr( "Open" ), this );
    m_saveAction = new QAction( QIcon( ":/resources/images/save.ico" ), tr( "Save" ), this );

    m_exportAction = new QAction( QIcon( ":/resources/images/export.ico" ), tr( "Export" ), this );

    m_grabAction = new QAction( QIcon( ":/resources/images/grab.ico" ), tr( "Grab" ), this );

    m_autoGrabAction = new QAction( QIcon( ":/resources/images/camera.ico" ), tr( "Autograb" ), this );
    m_autoGrabAction->setCheckable( true );
    m_autoGrabAction->setChecked( true );

    m_calculateAction = new QAction( QIcon( ":/resources/images/checkerflag.ico" ), tr( "Calculate" ), this );
    m_settingsAction = new QAction( QIcon( ":/resources/images/settings.ico" ), tr( "Settings" ), this );
    m_exitAction = new QAction( QIcon( ":/resources/images/power.ico" ), tr( "Exit" ), this );
    m_aboutAction = new QAction( QIcon( ":/resources/images/help.ico" ), tr( "About" ), this );

    connect( m_grabAction, &QAction::triggered, this, &MainWindow::grabFrame );
    connect( m_calculateAction, &QAction::triggered, this, &MainWindow::calculate );
    connect( m_settingsAction, &QAction::triggered, this, &MainWindow::settingsDialog );
    connect( m_exitAction, &QAction::triggered, this, &MainWindow::close );

}

void MainWindow::setupMenus()
{
    m_menuBar = new QMenuBar(this);

    auto fileMenu = m_menuBar->addMenu( tr( "File" ) );
    fileMenu->addAction( m_newAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_openAction );
    fileMenu->addAction( m_saveAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_exportAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_exitAction );

    auto actionsMenu = m_menuBar->addMenu( tr( "Actions" ) );
    actionsMenu->addAction( m_grabAction );
    actionsMenu->addAction( m_autoGrabAction );
    actionsMenu->addSeparator();
    actionsMenu->addAction( m_calculateAction );
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

    m_toolBar->addAction( m_newAction );
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
    if (m_autoGrabAction->isChecked())
        grabFrame();
}

void MainWindow::grabFrame()
{
    m_widget->grabFrame();
}

void MainWindow::calculate()
{
    m_widget->calculate();
}

void MainWindow::settingsDialog()
{
}
