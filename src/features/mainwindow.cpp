#include "src/common/precompiled.h"

#include "mainwindow.h"

#include "documentwidget.h"

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

FeaturesDocument *MainWindow::currentDocument() const
{
    return getCurrentDocument< FeaturesDocument >();
}

void MainWindow::addDocument()
{
    DocumentMainWindow::addDocument( new FeaturesDocument( this ) );
}

void MainWindow::importDialog()
{
    auto doc = currentDocument();

    if ( doc )
        doc->importDialog();

}

void MainWindow::setupActions()
{
    m_newDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New document" ), this );

    m_importAction = new QAction( QIcon( ":/resources/images/import.ico" ), tr( "Import" ), this );

    m_exitAction = new QAction( QIcon( ":/resources/images/power.ico" ), tr( "Exit" ), this );
    m_aboutAction = new QAction( QIcon( ":/resources/images/help.ico" ), tr( "About" ), this );

    connect( m_newDocumentAction, &QAction::triggered, this, &MainWindow::addDocument );
    connect( m_importAction, &QAction::triggered, this, &MainWindow::importDialog );
    connect( m_exitAction, &QAction::triggered, this, &MainWindow::close );
}

void MainWindow::setupMenus()
{
    m_menuBar = new QMenuBar(this);

    auto fileMenu = m_menuBar->addMenu( tr( "File" ) );
    fileMenu->addAction( m_newDocumentAction );
    fileMenu->addSeparator();
    fileMenu->addAction( m_importAction );
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
    m_toolBar->addAction( m_newDocumentAction );
    m_toolBar->addSeparator();
    m_toolBar->addAction( m_importAction );

    addToolBar( m_toolBar );
}
