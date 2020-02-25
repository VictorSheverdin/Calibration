#pragma once

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
class MonocularImageCalibrationDocument;
class StereoImageCalibrationDocument;
class TrippleCalibrationDocument;
class ReportDocumentBase;
class MonocularReportDocument;
class StereoReportDocument;

class MainWindow : public DocumentMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( QWidget *parent = nullptr );
    explicit MainWindow( const QString &cameraIp, QWidget *parent = nullptr );
    explicit MainWindow( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent = nullptr );

    QPointer< MonocularCameraCalibrationDocument > addMonocularCameraCalibrationDocument( const QString &cameraIp );
    QPointer< StereoCameraCalibrationDocument > addStereoCameraCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp );

    QPointer< MonocularImageCalibrationDocument > addMonocularCalibrationDocument();
    QPointer< StereoImageCalibrationDocument > addStereoCalibrationDocument();

    QPointer< MonocularReportDocument > addMonocularReportDocument();
    QPointer< StereoReportDocument > addStereoReportDocument();

    CalibrationDocumentBase *currentCalibrationDocument() const;
    CameraCalibrationDocumentBase *currentCameraCalibrationDocument() const;
    MonocularCameraCalibrationDocument *currentMonocularCalibrationDocument() const;
    StereoCameraCalibrationDocument *currentStereoCalibrationDocument() const;
    TrippleCalibrationDocument *currentTrippleCalibrationDocument() const;
    ReportDocumentBase *currentReportDocument() const;

public slots:
    void importDialog();
    void exportDialog();

    void grabFrame();

    void exportYamlResults();

    void calculate();

    void settingsDialog();

    void clearIcons();

    void choiceCalibrationDialog();

    void addMonocularImageCalibrationDialog();
    void addStereoImageCalibrationDialog();

    void addMonocularCameraCalibrationDialog();
    void addStereoCameraCalibrationDialog();

protected:
    QPointer< QMenuBar > m_menuBar;

    QPointer< QAction > m_newCalibrationDocumentAction;
    QPointer< QAction > m_openAction;
    QPointer< QAction > m_saveAction;

    QPointer< QAction > m_importAction;
    QPointer< QAction > m_exportAction;

    QPointer< QAction > m_grabAction;
    QPointer< QAction > m_autoGrabAction;
    QPointer< QAction > m_exportYamlAction;
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
