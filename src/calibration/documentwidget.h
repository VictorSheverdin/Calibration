#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

class MonocularCalibrationData;
class StereoCalibrationData;

class CalibrationWidgetBase;
class MonocularCameraCalibrationWidget;
class StereoCameraCalibrationWidget;

class CalibrationDocumentBase;
class MonocularCalibrationWidgetBase;
class StereoCalibrationWidgetBase;
class CameraCalibrationWidgetBase;
class MonocularImageCalibrationWidget;
class StereoImageCalibrationWidget;
class MonocularCameraCalibrationDocument;
class StereoCameraCalibrationDocument;
class TrippleCalibrationDocument;
class MonocularReportWidget;
class StereoReportWidget;

class ReportDocumentBase;

class CalibrationDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit CalibrationDocumentBase( QWidget* parent = nullptr );

    CalibrationWidgetBase *widget() const;

    MonocularCameraCalibrationDocument *toMonocularCalibrationDocument();
    const MonocularCameraCalibrationDocument *toMonocularCalibrationDocument() const;
    bool isMonocularCalibrationDocument() const;

    StereoCameraCalibrationDocument *toStereoCalibrationDocument();
    const StereoCameraCalibrationDocument *toStereoCalibrationDocument() const;
    bool isStereoCalibrationDocument() const;

    TrippleCalibrationDocument *toTrippleCalibrationDocument();
    const TrippleCalibrationDocument *toTrippleCalibrationDocument() const;
    bool isTrippleCalibrationDocument() const;

    ReportDocumentBase *toReportDocument();
    const ReportDocumentBase *toReportDocument() const;
    bool isReportDocument() const;

public slots:
    void importDialog();
    void exportDialog();

    void calculate();

    void clearIcons();

private:
    void initialize();

};

class CameraCalibrationDocumentBase : public CalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit CameraCalibrationDocumentBase( QWidget* parent = nullptr );

    CalibrationWidgetBase *widget() const;

public slots:
    void importDialog();
    void exportDialog();

    void grabFrame();
    void calculate();

    void clearIcons();

private:
    void initialize();

};

class MonocularImageCalibrationDocument : public CalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit MonocularImageCalibrationDocument( QWidget* parent = nullptr );

    MonocularImageCalibrationWidget *widget() const;

private:
    void initialize();

};

class StereoImageCalibrationDocument : public CalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit StereoImageCalibrationDocument( QWidget* parent = nullptr );

    StereoImageCalibrationWidget *widget() const;

private:
    void initialize();

};

class MonocularCameraCalibrationDocument : public CameraCalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit MonocularCameraCalibrationDocument( const QString &cameraIp, QWidget* parent = nullptr );

    MonocularCameraCalibrationWidget *widget() const;

private:
    void initialize( const QString &cameraIp );

};

class StereoCameraCalibrationDocument : public CameraCalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit StereoCameraCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

    StereoCameraCalibrationWidget *widget() const;

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};

class TrippleCalibrationDocument : public CameraCalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit TrippleCalibrationDocument( QWidget* parent = nullptr );

private:
    void initialize();

};

class ReportDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit ReportDocumentBase( QWidget* parent = nullptr );

    virtual void exportYaml() const = 0;

private:
    void initialize();

};

class MonocularReportDocument : public ReportDocumentBase
{
    Q_OBJECT

public:
    explicit MonocularReportDocument( QWidget* parent = nullptr );

    MonocularReportWidget *widget() const;

    void report( const MonocularCalibrationData &calibration );

    virtual void exportYaml() const override;

private:
    void initialize();

};

class StereoReportDocument : public ReportDocumentBase
{
    Q_OBJECT

public:
    explicit StereoReportDocument( QWidget* parent = nullptr );

    StereoReportWidget *widget() const;

    void report( const StereoCalibrationData &calibration );

    virtual void exportYaml() const override;

private:
    void initialize();

};
