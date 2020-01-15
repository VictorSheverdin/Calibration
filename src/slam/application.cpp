#include "src/common/precompiled.h"

#include "application.h"

#include "mainwindow.h"

#include "slamgeometry.h"
#include "src/common/image.h"

#include "src/common/functions.h"

Application::Application( int &argc, char **argv )
    : QApplication(argc, argv)
{
    initialize( argc, argv );
}

Application::~Application()
{
}

void Application::initialize( int &, char ** )
{
    qRegisterMetaType< CvImage >();
    qRegisterMetaType< SlamGeometry >();

    QFile cssFile(":/resources/qss/style.css");

    if ( cssFile.open( QIODevice::ReadOnly ) ) {
        QString cssString( cssFile.readAll() );
        setStyleSheet( cssString );
    }
    else
        QMessageBox::critical( nullptr, tr( "Error"), tr( "Can't load css file:" ) + cssFile.fileName() );

    setWindowIcon( QIcon( ":/resources/images/slam.ico" ) );

    m_mainWindow = new MainWindow();

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
