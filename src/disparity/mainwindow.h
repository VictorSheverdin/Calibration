#pragma once

#include <QMainWindow>

#include <QPointer>

#include "src/common/defs.h"
#include "src/common/supportwidgets.h"

class CameraDisparityWidget;
class DisparityDocumentBase;
class CameraDisparityDocument;
class DiskDisparityDocument;
class StereoDisparityDocument;
class FileDisparityDocument;

class MainWindow : public DocumentMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );
    explicit MainWindow( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent = nullptr );

    void addStereoDisparityDocument();
    void addFileDisparityDocument();
    void addCameraDisparityDocument( const QString &leftCameraIp, const QString &rightCameraIp );

    DisparityDocumentBase *currentDisparityDocument() const;
    DiskDisparityDocument *currentDiskDisparityDocument() const;
    CameraDisparityDocument *currentCameraDisparityDocument() const;
    StereoDisparityDocument *currentStereoDisparityDocument() const;
    FileDisparityDocument *currentFileDisparityDocument() const;

public slots:
    void choiceDisparityDialog();

    void addStereoDisparity();
    void addFileDisparity();
    void addCameraDisparityDialog();

    void loadCalibrationDialog();

    void importDialog();
    void exportDialog();

    void clearIcons();
    void settingsDialog();

    void setStatusBarText( const QString &text );

protected:
    QPointer< QMenuBar > m_menuBar;

    QPointer< QAction > m_newDisparityDocumentAction;
    QPointer< QAction > m_loadCalibrationAction;

    QPointer< QAction > m_importAction;
    QPointer< QAction > m_exportAction;

    QPointer< QAction > m_clearIconsAction;

    QPointer< QAction > m_settingsAction;

    QPointer< QAction > m_exitAction;

    QPointer< QAction > m_aboutAction;

    QPointer< QToolBar > m_toolBar;

    void setupActions();
    void setupMenus();
    void setupToolBars();

private:
    void initialize();

};

