#include "src/common/precompiled.h"

#include "mainwindow.h"

#include "disparitypreviewwidget.h"

MainWindow::MainWindow( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent )
    : QMainWindow(parent)
{
    initialize( leftCameraIp, rightCameraIp );
}

void MainWindow::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    m_widget = new PreviewWidget( leftCameraIp, rightCameraIp, this );

    initialize();
}

void MainWindow::initialize()
{
    setCentralWidget( m_widget );

    setAttribute( Qt::WA_DeleteOnClose );

}

void MainWindow::loadCalibrationFile( const std::string &fileName )
{
    m_widget->loadCalibrationFile( fileName );
}
