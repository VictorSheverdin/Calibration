#include "precompiled.h"

#include "application.h"

#include "mainwindow.h"

Application::Application( int &argc, char **argv )
    : QApplication(argc, argv)
{
    initialize( argc, argv );
}

void Application::initialize( int &argc, char **argv )
{
    QFile cssFile(":/resources/qss/style.css");

    if ( cssFile.open( QIODevice::ReadOnly ) ) {
        QString cssString( cssFile.readAll() );
        setStyleSheet( cssString );
    }
    else
        QMessageBox::critical( nullptr, tr( "Error"), tr( "Can't load css file:" ) + cssFile.fileName() );

    setWindowIcon( QIcon( ":/resources/images/checkerboard.ico" ) );

    m_mainWindow = new MainWindow( "169.254.142.79" , "169.254.171.30" );
    mainWindow()->setCameraDecimation( CameraWidgetBase::WHOLE );
    m_mainWindow->showMaximized();

}

MainWindow *Application::mainWindow() const
{
    return m_mainWindow;
}

Application *application()
{
    return dynamic_cast< Application * >( qApp );
}
