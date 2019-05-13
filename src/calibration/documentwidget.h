#pragma once

#include <QWidget>

class QVBoxLayout;

class MonocularCalibrationDocument;
class StereoCalibrationDocument;
class TrippleCalibrationDocument;
class ReportDocument;

class DocumentBase : public QWidget
{
    Q_OBJECT

public:
    explicit DocumentBase( QWidget *widget, QWidget* parent = nullptr );

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

    void setWidget(QWidget *widget );

protected:
    QVBoxLayout *m_layout;

    QWidget *m_widget;

private:
    void initialize();

};

class MonocularCalibrationDocument : public DocumentBase
{
    Q_OBJECT

public:
    explicit MonocularCalibrationDocument( const std::string &cameraIp, QWidget* parent = nullptr );

private:
    void initialize( const std::string &cameraIp );

};

class StereoCalibrationDocument : public DocumentBase
{
    Q_OBJECT

public:
    explicit StereoCalibrationDocument( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent = nullptr );

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};

class TrippleCalibrationDocument : public DocumentBase
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
