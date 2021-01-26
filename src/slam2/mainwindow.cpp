#include "src/common/precompiled.h"

#include "mainwindow.h"

#include "slamdialog.h"
#include "slamdocument.h"

MainWindow::MainWindow( QWidget *parent )
    : DocumentMainWindow( parent )
{
    initialize();
}

void MainWindow::initialize()
{
    setupActions();
    setupMenus();
    setupToolBars();
}

void MainWindow::setupActions()
{
    _newAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New document" ), this );

    _exitAction = new QAction( QIcon( ":/resources/images/power.ico" ), tr( "Exit" ), this );
    _aboutAction = new QAction( QIcon( ":/resources/images/help.ico" ), tr( "About" ), this );

    connect( _newAction, &QAction::triggered, this, &MainWindow::addDialog );
    connect( _exitAction, &QAction::triggered, this, &MainWindow::close );
}

void MainWindow::setupMenus()
{
    _menuBar = new QMenuBar(this);

    auto fileMenu = _menuBar->addMenu( tr( "File" ) );
    fileMenu->addAction( _newAction );
    fileMenu->addSeparator();
    fileMenu->addAction( _exitAction );

    auto helpMenu = _menuBar->addMenu( tr( "Help" ) );
    helpMenu->addAction( _aboutAction );

    setMenuBar( _menuBar );
}

void MainWindow::setupToolBars()
{
    // Настройки панели инструментов проекта
    _toolBar = new QToolBar( tr( "Project tool bar" ), this );
    _toolBar->addAction( _newAction );
    _toolBar->addSeparator();

    addToolBar( _toolBar );
}

void MainWindow::addDialog()
{
    ImagesDialog dialog( this );

    if ( dialog.exec() == DialogBase::Accepted )
        addDocument( dialog.leftFileNames(), dialog.rightFileNames(), dialog.calibrationFile() );
}

void MainWindow::addDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile )
{
    DocumentMainWindow::addDocument( new ImageSlamDocument( leftList, rightList, calibrationFile, this ) );
}

