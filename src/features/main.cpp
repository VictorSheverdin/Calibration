#include "src/common/precompiled.h"

#include "application.h"

#include <QVTKOpenGLNativeWidget.h>

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat( QVTKOpenGLNativeWidget::defaultFormat() );

    Application a(argc, argv);

    return a.exec();
}

