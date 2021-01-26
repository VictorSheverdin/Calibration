#include "src/common/precompiled.h"

#include "application.h"

#include "mainwindow.h"

Application::Application( int &argc, char **argv )
    : QApplication( argc, argv )
{
    initialize( argc, argv );
}

Application::~Application()
{
}

void Application::initialize( int &, char ** )
{
    QFile cssFile(":/resources/qss/style.css");

    if ( cssFile.open( QIODevice::ReadOnly ) ) {
        QString cssString( cssFile.readAll() );
        setStyleSheet( cssString );
    }
    else
        QMessageBox::critical( nullptr, tr( "Error"), tr( "Can't load css file:" ) + cssFile.fileName() );

    setWindowIcon( QIcon( ":/resources/images/slam.ico" ) );

    _mainWindow = new MainWindow();

    _mainWindow->showMaximized();

}

MainWindow *Application::mainWindow() const
{
    return _mainWindow;
}

Application *application()
{
    return dynamic_cast< Application * >( qApp );
}
