#include "src/common/precompiled.h"

#include "application.h"

#include "mainwindow.h"

#include "slamgeometry.h"
#include "src/common/image.h"

#include "src/common/functions.h"

#include "src/common/vimbacamera.h"

Application::Application( int &argc, char **argv )
    : QApplication( argc, argv ), m_vimbaSystem( AVT::VmbAPI::VimbaSystem::GetInstance() )
{
    initialize( argc, argv );
}

Application::~Application()
{
}

void Application::initialize( int &, char ** )
{
    checkVimbaStatus( m_vimbaSystem.Startup(), "Could not start Vimba system" );

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

AVT::VmbAPI::VimbaSystem &Application::vimbaSystem() const
{
    return m_vimbaSystem;
}

MainWindow *Application::mainWindow() const
{
    return m_mainWindow;
}

Application *application()
{
    return dynamic_cast< Application * >( qApp );
}
