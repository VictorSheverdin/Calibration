#include "precompiled.h"

#include "disparitypreviewwidget.h"

int main(int argc, char** argv)
{
    QApplication a(argc, argv);

    PreviewWidget w( 0, 2 );
    w.resize( 1400, 900 );
    w.showMaximized();

    return a.exec();
}
