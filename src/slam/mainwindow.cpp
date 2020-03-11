#include "src/common/precompiled.h"

#include "mainwindow.h"

#include "slamdocument.h"

#include "slamdialog.h"
#include "choicedialog.h"

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

    addImagesDocument();
}


void MainWindow::choiceDialog()
{
     ChoiceDialog dlg( this );

     if ( dlg.exec() == DialogBase::Accepted ) {
         switch ( dlg.selectedType() ) {

         case ChoiceDialog::IMAGES:
             addImageSlamDialog();
             break;

         case ChoiceDialog::CAMERA:
             addCameraSlamDialog();
             break;

         default:
             break;

         }

     }

}

void MainWindow::addImageSlamDialog()
{
    ImagesDialog dialog( this );

    if ( dialog.exec() == DialogBase::Accepted )
        addImagesDocument();
}

void MainWindow::addCameraSlamDialog()
{
    CamerasDialog dialog( this );

    if ( dialog.exec() == DialogBase::Accepted )
        addCamerasDocument();
}

void MainWindow::addImagesDocument()
{
    addDocument( new ImageSlamDocument( "/home/victor/Polygon/calibration.yaml", this ) );
}

void MainWindow::addCamerasDocument()
{
}

void MainWindow::setupActions()
{
    m_newSlamDocumentAction = new QAction( QIcon( ":/resources/images/new.ico" ), tr( "New SLAM document" ), this );

    m_exitAction = new QAction( QIcon( ":/resources/images/power.ico" ), tr( "Exit" ), this );
    m_aboutAction = new QAction( QIcon( ":/resources/images/help.ico" ), tr( "About" ), this );

    connect( m_newSlamDocumentAction, &QAction::triggered, this, &MainWindow::choiceDialog );
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
