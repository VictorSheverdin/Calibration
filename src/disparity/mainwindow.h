#pragma once

#include <QMainWindow>

#include <QPointer>

#include "src/common/defs.h"
#include "src/common/supportwidgets.h"

class CameraDisparityWidget;
class DisparityDocumentBase;
class CameraDisparityDocument;
class ImageDisparityDocument;

class MainWindow : public MainWindowBase
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );
    explicit MainWindow( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent = nullptr );

    void addImageDisparityDocument();
    void addCameraDisparityDocument( const QString &leftCameraIp, const QString &rightCameraIp );

    DisparityDocumentBase *currentDisparityDocument() const;
    CameraDisparityDocument *currentCameraDisparityDocument() const;
    ImageDisparityDocument *currentImageDisparityDocument() const;

public slots:
    void choiceDisparityDialog();

    void addImageDisparity();
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

