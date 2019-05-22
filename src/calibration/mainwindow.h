#pragma once

#include <QMainWindow>

#include <QPointer>

#include "src/common/defs.h"

#include "src/common/supportwidgets.h"
#include "src/common/documentarea.h"

class CalibrationWidgetBase;
class DocumentArea;

class DocumentBase;
class CalibrationDocumentBase;
class CameraCalibrationDocumentBase;
class MonocularCameraCalibrationDocument;
class StereoCameraCalibrationDocument;
class TrippleCalibrationDocument;
class ReportDocument;

class MainWindow : public MainWindowBase
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );
    explicit MainWindow( const QString &cameraIp, QWidget *parent = nullptr );
    explicit MainWindow( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent = nullptr );

    void addMonocularCameraCalibrationDocument( const QString &cameraIp );
    void addStereoCameraCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp );

    void addMonocularCalibrationDocument();
    void addStereoCalibrationDocument();

    CalibrationDocumentBase *currentCalibrationDocument() const;
    CameraCalibrationDocumentBase *currentCameraCalibrationDocument() const;
    MonocularCameraCalibrationDocument *currentMonocularCalibrationDocument() const;
    StereoCameraCalibrationDocument *currentStereoCalibrationDocument() const;
    TrippleCalibrationDocument *currentTrippleCalibrationDocument() const;
    ReportDocument *currentReportDocument() const;

public slots:
    void importDialog();
    void exportDialog();

    void grabFrame();
    void calculate();

    void settingsDialog();

    void clearIcons();

    void addMonocularCalibrationDialog();
    void addStereoCalibrationDialog();

    void addMonocularCameraCalibrationDialog();
    void addStereoCameraCalibrationDialog();

protected:
    QPointer< QMenuBar > m_menuBar;

    QPointer< QAction > m_newMonocularImageDocumentAction;
    QPointer< QAction > m_newStereoImageDocumentAction;
    QPointer< QAction > m_newMonocularCameraDocumentAction;
    QPointer< QAction > m_newStereoCameraDocumentAction;
    QPointer< QAction > m_openAction;
    QPointer< QAction > m_saveAction;

    QPointer< QAction > m_importAction;
    QPointer< QAction > m_exportAction;

    QPointer< QAction > m_grabAction;
    QPointer< QAction > m_autoGrabAction;
    QPointer< QAction > m_calculateAction;
    QPointer< QAction > m_clearIconsAction;

    QPointer< QAction > m_settingsAction;

    QPointer< QAction > m_exitAction;

    QPointer< QAction > m_aboutAction;

    QPointer< QToolBar > m_toolBar;

    void setupActions();
    void setupMenus();
    void setupToolBars();

    static const int m_grabInterval = 1500;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize( const QString &cameraIp );
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );
    void initialize();

};
