#pragma once

#include <QMainWindow>

#include <QPointer>

#include "src/common/defs.h"

class CalibrationWidgetBase;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( const std::string &cameraIp, QWidget *parent = nullptr );
    explicit MainWindow( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent = nullptr );

public slots:
    void grabFrame();
    void calculate();

    void settingsDialog();

    void setCameraDecimation( VimbaDecimationType type );

    void clearIcons();

protected:
    QPointer<CalibrationWidgetBase> m_widget;

    QPointer<QMenuBar> m_menuBar;
    QPointer<QStatusBar> m_statusBar;

    QPointer<QAction> m_newAction;
    QPointer<QAction> m_openAction;
    QPointer<QAction> m_saveAction;

    QPointer<QAction> m_exportAction;

    QPointer<QAction> m_grabAction;
    QPointer<QAction> m_autoGrabAction;
    QPointer<QAction> m_calculateAction;

    QPointer<QAction> m_settingsAction;

    QPointer<QAction> m_exitAction;

    QPointer<QAction> m_aboutAction;

    QPointer< QToolBar > m_toolBar;

    void setupActions();
    void setupMenus();
    void setupToolBars();
    void setupStatusBar();

    static const int m_grabInterval = 1500;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize(const std::string &cameraIp );
    void initialize(const std::string &leftCameraIp, const std::string &rightCameraIp );
    void initialize();

};
