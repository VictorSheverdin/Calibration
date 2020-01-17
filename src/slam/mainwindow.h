#pragma once

#include <QPointer>

#include "src/common/supportwidgets.h"

class SlamWidget;

class MainWindow : public DocumentMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );

public slots:

protected:
    QPointer< QAction > m_newSlamDocumentAction;
    QPointer< QAction > m_exitAction;
    QPointer< QAction > m_aboutAction;

    QPointer< QMenuBar > m_menuBar;

    QPointer< QToolBar > m_toolBar;

    void setupActions();
    void setupMenus();
    void setupToolBars();

private:
    void initialize();

};
