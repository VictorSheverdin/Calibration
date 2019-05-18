#pragma once

#include <QWidget>

class QVBoxLayout;

class CalibrationWidgetBase;
class MonocularCalibrationWidget;
class StereoCalibrationWidget;

class CalibrationDocumentBase;
class MonocularCalibrationDocument;
class StereoCalibrationDocument;
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

    MonocularCalibrationDocument *toMonocularCalibrationDocument();
    const MonocularCalibrationDocument *toMonocularCalibrationDocument() const;
    bool isMonocularCalibrationDocument() const;

    StereoCalibrationDocument *toStereoCalibrationDocument();
    const StereoCalibrationDocument *toStereoCalibrationDocument() const;
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
    explicit MonocularCalibrationDocument( const QString &cameraIp, QWidget* parent = nullptr );

    MonocularCalibrationWidget *widget() const;

private:
    void initialize( const QString &cameraIp );

};

class StereoCalibrationDocument : public CalibrationDocumentBase
{
    Q_OBJECT

public:
    explicit StereoCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

    StereoCalibrationWidget *widget() const;

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};

class TrippleCalibrationDocument : public CalibrationDocumentBase
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
