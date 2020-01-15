#pragma once

#include <QPointer>

#include "src/common/supportwidgets.h"

class SlamWidget;

class MainWindow : public MainWindowBase
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );

public slots:

protected:
    QPointer< SlamWidget > m_displayWidget;

private:
    void initialize();

};
