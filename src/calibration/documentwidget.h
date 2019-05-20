#pragma once

#include <QWidget>

class QVBoxLayout;

class CalibrationWidgetBase;
class MonocularCameraCalibrationWidget;
class StereoCameraCalibrationWidget;

class CalibrationDocumentBase;
class MonocularCalibrationWidget;
class StereoCalibrationWidget;
class CameraCalibrationWidgetBase;
class MonocularCameraCalibrationDocument;
class StereoCameraCalibrationDocument;
class TrippleCalibrationDocument;
class ReportDocument;

class DocumentBase : public QWidget
{
    Q_OBJECT

public:
    explicit DocumentBase( QWidget *widget, QWidget* parent = nullptr );

    CalibrationDocumentBase *toCalibrationDocument();
    const CalibrationDocumentBase *toCalibrationDocument() const;
    bool isCalibrationDocument() const;

    MonocularCameraCalibrationDocument *toMonocularCalibrationDocument();
    const MonocularCameraCalibrationDocument *toMonocularCalibrationDocument() const;
    bool isMonocularCalibrationDocument() const;

    StereoCameraCalibrationDocument *toStereoCalibrationDocument();
    const StereoCameraCalibrationDocument *toStereoCalibrationDocument() const;
    bool isStereoCalibrationDocument() const;

    TrippleCalibrationDocument *toTrippleCalibrationDocument();
    const TrippleCalibrationDocument *toTrippleCalibrationDocument() const;
    bool isTrippleCalibrationDocument() const;

    ReportDocument *toReportDocument();
    const ReportDocument *toReportDocument() const;
    bool isReportDocument() const;

    void setWidget( QWidget *widget );
    QWidget *widget() const;

protected:
    QVBoxLayout *m_layout;

    QWidget *m_widget;

private:
    void initialize();

};

class CalibrationDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit CalibrationDocumentBase( QWidget* parent = nullptr );

    CalibrationWidgetBase *widget() const;

public slots:
    void importDialog();
    void exportDialog();

    void calculate();

    void clearIcons();

private:
    void initialize();

};

class CameraCalibrationDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit CameraCalibrationDocumentBase( QWidget* parent = nullptr );

    CameraCalibrationWidgetBase *widget() const;

public slots:
    void grabFrame();
    void calculate();

    void clearIcons();

private:
    void initialize();

};

class MonocularCalibrationDocument : public CalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit MonocularCalibrationDocument( QWidget* parent = nullptr );

    MonocularCalibrationWidget *widget() const;

private:
    void initialize();

};

class StereoCalibrationDocument : public CalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit StereoCalibrationDocument( QWidget* parent = nullptr );

    StereoCalibrationWidget *widget() const;

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

class ReportDocument : public DocumentBase
{
    Q_OBJECT

public:
    explicit ReportDocument( QWidget* parent = nullptr );

private:
    void initialize();

};
