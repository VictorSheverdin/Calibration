#include "precompiled.h"

#include "application.h"

#include "mainwindow.h"

#include "functions.h"

Application::Application( int &argc, char **argv )
    : QApplication(argc, argv), m_vimbaSystem( AVT::VmbAPI::VimbaSystem::GetInstance() )
{
    initialize( argc, argv );
}

Application::~Application()
{
    m_vimbaSystem.Shutdown();
}

void Application::initialize( int &argc, char **argv )
{
    checkVimbaStatus( m_vimbaSystem.Startup(), "Could not start Vimba system");

    QFile cssFile(":/resources/qss/style.css");

    if ( cssFile.open( QIODevice::ReadOnly ) ) {
        QString cssString( cssFile.readAll() );
        setStyleSheet( cssString );
    }
    else
        QMessageBox::critical( nullptr, tr( "Error"), tr( "Can't load css file:" ) + cssFile.fileName() );

    setWindowIcon( QIcon( ":/resources/images/checkerboard.ico" ) );

    m_mainWindow = new MainWindow( /*"169.254.142.79", */"169.254.171.30" );
    mainWindow()->setCameraDecimation( CameraWidgetBase::HALF );
    m_mainWindow->showMaximized();

}

MainWindow *Application::mainWindow() const
{
    return m_mainWindow;
}

AVT::VmbAPI::VimbaSystem &Application::vimbaSystem() const
{
    return m_vimbaSystem;
}

Application *application()
{
    return dynamic_cast< Application * >( qApp );
}
