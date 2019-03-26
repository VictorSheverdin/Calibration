#include "precompiled.h"

#include "application.h"

#include "mainwindow.h"

Application::Application( int &argc, char **argv )
    : QApplication(argc, argv)
{
    initialize();
}

void Application::initialize()
{
    QFile cssFile(":/resources/qss/style.css");

    if ( cssFile.open( QIODevice::ReadOnly ) ) {
        QString cssString( cssFile.readAll() );
        setStyleSheet( cssString );
    }
    else
        QMessageBox::critical( nullptr, tr( "Error"), tr( "Can't load css file:" ) + cssFile.fileName() );

    setWindowIcon( QIcon( ":/resources/images/checkerboard.ico" ) );

    (new MainWindow( 0, 2 ))->showMaximized();
}
