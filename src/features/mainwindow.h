#pragma once

#include <QPointer>

#include "src/common/supportwidgets.h"

class FeaturesDocument;

class MainWindow : public DocumentMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );

    FeaturesDocument *currentDocument() const;

public slots:
    void addDocument();

    void importDialog();

protected:
    QPointer< QAction > m_newDocumentAction;

    QPointer< QAction > m_importAction;

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
