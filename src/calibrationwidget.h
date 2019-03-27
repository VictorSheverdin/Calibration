#pragma once

#include <QSplitter>
#include <QPointer>

class MonocularTaskWidget;
class StereoTaskWidget;
class IconBase;
class IconsWidget;
class ImageWidget;
class ImageDialog;
class ReportDialog;

class CalibrationWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CalibrationWidgetBase( QWidget *parent = nullptr );

public slots:
    virtual void grabFrame() = 0;
    virtual void calculate() = 0;

protected slots:
    void showIcon( IconBase *icon );

protected:
    QPointer< IconsWidget > m_iconsWidget;
    QPointer< ImageDialog > m_iconViewDialog;
    QPointer< ReportDialog > m_reportDialog;

private:
    void initialize();

};

class MonocularCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    MonocularCalibrationWidget( const int cameraIndex, QWidget *parent = nullptr );

public slots:
    virtual void grabFrame() override;
    virtual void calculate() override;

protected:
    QPointer<MonocularTaskWidget> m_taskWidget;

private:
    void initialize( const int cameraIndex );

};

class StereoCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT
public:
    StereoCalibrationWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget *parent = nullptr );

public slots:
    virtual void grabFrame() override;
    virtual void calculate() override;

protected:
    QPointer<StereoTaskWidget> m_taskWidget;

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );

};
