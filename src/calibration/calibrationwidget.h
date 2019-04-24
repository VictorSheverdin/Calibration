#pragma once

#include <QSplitter>
#include <QPointer>

#include "templateprocessor.h"

#include "reportwidget.h"
#include "src/common/defs.h"

class TaskWidgetBase;
class MonocularTaskWidget;
class StereoTaskWidget;
class IconBase;
class IconsWidget;
class ImageWidget;
class ImageDialog;

class CalibrationWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CalibrationWidgetBase( QWidget *parent = nullptr );

    void setCameraDecimation( VimbaDecimationType type );

public slots:
    virtual void grabFrame() = 0;
    virtual void calculate() = 0;

protected slots:
    virtual void showIcon( IconBase *icon ) = 0;

protected:
    QPointer< IconsWidget > m_iconsWidget;
    QPointer< ImageDialog > m_iconViewDialog;

    TemplateProcessor m_processor;

    QPointer<TaskWidgetBase> m_taskWidget;

    static const int m_minimumCalibrationFrames = 5;

    MonocularCalibrationData calcMonocularCalibration( const std::vector< CvImage > &frames );

private:
    void initialize();

};

class MonocularCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    MonocularCalibrationWidget( const std::string &cameraIp, QWidget *parent = nullptr );

    MonocularTaskWidget *taskWidget() const;

public slots:
    virtual void grabFrame() override;
    virtual void calculate() override;

protected slots:
    virtual void showIcon( IconBase *icon ) override;

protected:
    QPointer< MonocularReportDialog > m_reportDialog;

private:
    void initialize( const std::string &cameraIp );

};

class StereoCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    StereoCalibrationWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent = nullptr );

    void saveXMLCalibration();

    StereoTaskWidget *taskWidget() const;

protected slots:
    virtual void showIcon( IconBase *icon ) override;

public slots:
    virtual void grabFrame() override;
    virtual void calculate() override;

protected:
    StereoCalibrationData calcStereoCalibration( const std::vector< CvImage > &leftFrames,
                                                   const std::vector< CvImage > &rightFrames );

    QPointer< StereoReportDialog > m_reportDialog;

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};
