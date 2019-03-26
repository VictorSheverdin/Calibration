#pragma once

#include <QSplitter>
#include <QPointer>

class MonocularTaskWidget;
class StereoTaskWidget;
class IconsWidget;

class CalibrationWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CalibrationWidgetBase( QWidget *parent = nullptr );

public slots:
    virtual void grabFrame() = 0;

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

protected:
    QPointer<MonocularTaskWidget> m_taskWidget;
    QPointer<IconsWidget> m_iconsWidget;

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

protected:
    QPointer<StereoTaskWidget> m_taskWidget;
    QPointer<IconsWidget> m_iconsWidget;

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );

};
