#pragma once

#include <QApplication>
#include <QPointer>

#include "VimbaCPP/Include/VimbaCPP.h"

class MainWindow;

class Application : public QApplication
{
    Q_OBJECT

public:
    Application( int &argc, char **argv );
    ~Application();

    MainWindow *mainWindow() const;

    AVT::VmbAPI::VimbaSystem &vimbaSystem() const;

protected:
    QPointer< MainWindow > m_mainWindow;

    AVT::VmbAPI::VimbaSystem &m_vimbaSystem;

private:
    void initialize(int &, char **);

};

Application *application();
