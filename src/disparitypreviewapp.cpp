#include "precompiled.h"

#include "disparitycontrolwidget.h"

int main(int argc, char** argv)
{
    QApplication a(argc, argv);

    DisparityControlWidget w;
    w.show();

    return a.exec();
}
