#pragma once

#include <QApplication>
#include <QPointer>

class MainWindow;

class Application : public QApplication
{
    Q_OBJECT

public:
    Application( int &argc, char **argv );
    ~Application();

    MainWindow *mainWindow() const;

protected:
    QPointer< MainWindow > _mainWindow;

private:
    void initialize(int &, char **);

};

Application *application();
