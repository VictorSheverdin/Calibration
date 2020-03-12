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
        addImagesDocument( dialog.leftFileNames(), dialog.rightFileNames(), dialog.calibrationFile() );
}

void MainWindow::addCameraSlamDialog()
{
    CamerasDialog dialog( this );

    if ( dialog.exec() == DialogBase::Accepted )
        addCamerasDocument( dialog.leftIp(), dialog.rightIp(), dialog.calibrationFile() );
}

void MainWindow::addImagesDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile )
{
    addDocument( new ImageSlamDocument( leftList, rightList, calibrationFile, this ) );
}

void MainWindow::addCamerasDocument( const QString &leftCameraIp, const QString &rightCameraIp, const QString &calibrationFile )
{
    addDocument( new CameraSlamDocument( leftCameraIp, rightCameraIp, calibrationFile, this ) );
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
