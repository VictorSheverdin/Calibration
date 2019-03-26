#pragma once

#include <QWidget>
#include <QPointer>

#include "image.h"

class QVBoxLayout;

class ParametersWidget;
class CameraWidgetBase;
class MonocularCameraWidget;
class StereoCameraWidget;

class TaskWidgetBase : public QWidget
{
    Q_OBJECT

public:
    explicit TaskWidgetBase( QWidget* parent = nullptr );

protected slots:
    void updateParameters();

protected:
    QPointer<QVBoxLayout> m_layout;
    QPointer<ParametersWidget> m_parametersWidget;
    QPointer<CameraWidgetBase> m_cameraWidget;

private:
    void initialize();

};

class MonocularTaskWidget : public TaskWidgetBase
{
    Q_OBJECT

public:
    explicit MonocularTaskWidget( const int cameraIndex, QWidget* parent = nullptr );

    MonocularCameraWidget *cameraWidget() const;

    const CvImage sourceImage() const;
    const CvImage previewImage() const;

private:
    void initialize (const int cameraIndex );

};


class StereoTaskWidget : public TaskWidgetBase
{
    Q_OBJECT

public:
    explicit StereoTaskWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent = nullptr );

    StereoCameraWidget *cameraWidget() const;

    const CvImage leftSourceImage() const;
    const CvImage leftDisplayedImage() const;

    const CvImage rightSourceImage() const;
    const CvImage rightDisplayedImage() const;

public slots:
    void setLeftSourceImage( const CvImage image );
    void setLeftDisplayedImage( const CvImage image );

    void setRightSourceImage( const CvImage image );
    void setRightDisplayedImage( const CvImage image );

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );

};


