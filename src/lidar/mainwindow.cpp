#include "src/common/precompiled.h"

#include "mainwindow.h"

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

    addSlamDialog();
}


void MainWindow::addSlamDialog()
{
/*    CamerasDialog dialog( this );

    if ( dialog.exec() == DialogBase::Accepted )
        addCamerasDocument();*/
}

void MainWindow::addSlamDocument()
{
}

void MainWindow::setupActions()
{
    m_newSlamDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New SLAM document" ), this );

    m_exitAction = new QAction( QIcon( ":/resources/images/power.ico" ), tr( "Exit" ), this );
    m_aboutAction = new QAction( QIcon( ":/resources/images/help.ico" ), tr( "About" ), this );

    connect( m_newSlamDocumentAction, &QAction::triggered, this, &MainWindow::addSlamDialog );
    connect( m_exitAction, &QAction::triggered, this, &MainWindow::close );
}

void MainWindow::setupMenus()
{
    m_menuBar = new QMenuBar(this);

    auto fileMenu = m_menuBar->addMenu( tr( "File" ) );
    fileMenu->addAction( m_newSlamDocumentAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_exitAction );

    auto helpMenu = m_menuBar->addMenu( tr( "Help" ) );
    helpMenu->addAction( m_aboutAction );

    setMenuBar(m_menuBar);
}

void MainWindow::setupToolBars()
{
    // Настройки панели инструментов проекта
    m_toolBar = new QToolBar( tr( "Project tool bar" ), this );
    m_toolBar->addAction( m_newSlamDocumentAction );
    m_toolBar->addSeparator();

    addToolBar( m_toolBar );
}
