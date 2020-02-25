#pragma once

#include <QTextEdit>
#include <QDialog>

#include "calibrationdata.h"

class CalibrationIconBase;
class QVBoxLayout;
class QToolBar;

class ReportWidget : public QTextEdit
{
    Q_OBJECT

public:
    explicit ReportWidget( QWidget* parent = nullptr );

    void addText( const QString &text );
    void addBreak();
    void addDoubleBreak();
    void addSpace( const int num = 1 );
    template <class T>
    void addNumber( const T num ) { addText( QString::number( num ) ); }
    void addIcon( const CalibrationIconBase& icon );
    void addImage( const CvImage& image );
    void addSize( const cv::Size& size );
    void addMatrix( const cv::Mat& mat );
    void addRect( const cv::Rect& rect );

protected:
    static const int m_reportFrameSize = 800;

private:
    void initialize();

};

class MonocularReportWidget : public ReportWidget
{
    Q_OBJECT

public:
    explicit MonocularReportWidget( QWidget *parent = nullptr );

    void report( const MonocularCalibrationData &calibration );
    const MonocularCalibrationData &calibrationResults() const;

public slots:
    void saveYAMLDialog();
    void loadYAMLDialog();

protected:
    MonocularCalibrationData m_calibrationResults;

};

class StereoReportWidget : public ReportWidget
{
    Q_OBJECT

public:
    explicit StereoReportWidget( QWidget *parent = nullptr );

    void report( const StereoCalibrationData &calibration );
    const StereoCalibrationData &calibrationResults() const;

public slots:
    void saveYAMLDialog();
    void loadYAMLDialog();

protected:
    StereoCalibrationData m_calibrationResults;

};


