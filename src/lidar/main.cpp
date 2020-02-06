#include "src/common/precompiled.h"

#include "application.h"

#include <QVTKOpenGLWidget.h>

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat( QVTKOpenGLWidget::defaultFormat() );

    Application a(argc, argv);

    return a.exec();
}

