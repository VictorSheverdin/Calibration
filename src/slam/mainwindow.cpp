#include "src/common/precompiled.h"

#include "mainwindow.h"

#include "slamwidget.h"

MainWindow::MainWindow( QWidget *parent )
    : MainWindowBase( parent )
{
    initialize();
}

void MainWindow::initialize()
{
    m_displayWidget = new SlamWidget( this );

    setCentralWidget( m_displayWidget );

}
