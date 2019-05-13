#pragma once

#include <QMainWindow>

#include <QPointer>

#include "src/common/defs.h"

#include "documentarea.h"

class CalibrationWidgetBase;
class DocumentArea;

class DocumentBase;
class MonocularCalibrationDocument;
class StereoCalibrationDocument;
class TrippleCalibrationDocument;
class ReportDocument;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( const std::string &cameraIp, QWidget *parent = nullptr );
    explicit MainWindow( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent = nullptr );

    void addDocument( DocumentBase *document );
    void addMonocularCalibrationDocument( const std::string &cameraIp );
    void addStereoCalibrationDocument( const std::string &leftCameraIp, const std::string &rightCameraIp );

    MonocularCalibrationDocument *currentMonocularCalibrationDocument() const;
    StereoCalibrationDocument *currentStereoCalibrationDocument() const;
    TrippleCalibrationDocument *currentTrippleCalibrationDocument() const;
    ReportDocument *currentReportDocument() const;

public slots:
    void grabFrame();
    void calculate();

    void settingsDialog();

    void clearIcons();

protected:
    QPointer<CalibrationWidgetBase> m_widget;

    QPointer< QMenuBar > m_menuBar;
    QPointer< QStatusBar > m_statusBar;

    QPointer< DocumentArea > m_documentArea;

    QPointer< QAction > m_newAction;
    QPointer< QAction > m_openAction;
    QPointer< QAction > m_saveAction;

    QPointer< QAction > m_exportAction;

    QPointer< QAction > m_grabAction;
    QPointer< QAction > m_autoGrabAction;
    QPointer< QAction > m_calculateAction;

    QPointer< QAction > m_settingsAction;

    QPointer< QAction > m_exitAction;

    QPointer< QAction > m_aboutAction;

    QPointer< QToolBar > m_toolBar;

    void setupDocuments();
    void setupActions();
    void setupMenus();
    void setupToolBars();
    void setupStatusBar();

    static const int m_grabInterval = 1500;

    virtual void timerEvent( QTimerEvent * ) override;

    template <class T> T* getCurrentDocument() const;

private:
    void initialize(const std::string &cameraIp );
    void initialize(const std::string &leftCameraIp, const std::string &rightCameraIp );
    void initialize();

};

#include "mainwindow.inl"
