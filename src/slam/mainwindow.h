#pragma once

#include <QPointer>

#include "src/common/supportwidgets.h"

class MainWindow : public DocumentMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );

public slots:
    void choiceDialog();

    void addImageSlamDialog();
    void addCameraSlamDialog();

protected:
    QPointer< QAction > m_newSlamDocumentAction;
    QPointer< QAction > m_newImuDocumentAction;
    QPointer< QAction > m_exitAction;
    QPointer< QAction > m_aboutAction;

    QPointer< QMenuBar > m_menuBar;

    QPointer< QToolBar > m_toolBar;

    void addImagesDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile );
    void addCamerasDocument( const QString &leftCameraIp, const QString &rightCameraIp, const QString &calibrationFile );
    void addImuDocument();

    void setupActions();
    void setupMenus();
    void setupToolBars();

private:
    void initialize();

};
