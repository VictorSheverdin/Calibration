#pragma once

#include <QPointer>

#include "src/common/supportwidgets.h"

class MainWindow : public DocumentMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );

public slots:
    void addDialog();

protected:
    QPointer< QAction > _newAction;
    QPointer< QAction > _exitAction;
    QPointer< QAction > _aboutAction;

    QPointer< QMenuBar > _menuBar;

    QPointer< QToolBar > _toolBar;

    void setupActions();
    void setupMenus();
    void setupToolBars();

    void addDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile );

private:
    void initialize();

};
