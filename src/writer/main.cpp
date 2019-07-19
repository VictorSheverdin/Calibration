#include "src/common/precompiled.h"

#include "writer.h"
#include "src/common/functions.h"

int main(int argc, char** argv)
{
    QApplication a(argc, argv);

    checkVimbaStatus( AVT::VmbAPI::VimbaSystem::GetInstance().Startup(), "Could not start Vimba system");

    Writer w( "192.168.80.82", "192.168.80.66" );

    auto res = a.exec();

    AVT::VmbAPI::VimbaSystem::GetInstance().Shutdown();

    return res;
}
