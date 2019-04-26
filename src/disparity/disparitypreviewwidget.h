#pragma once

#include <QWidget>
#include <QSplitter>

#include "src/common/defs.h"
#include "stereoprocessor.h"

#include "VimbaCPP/Include/VimbaCPP.h"

#include "src/common/calibrationdata.h"

class ImageWidget;
class DisparityControlWidget;
class PCLViewer;
class BMControlWidget;

class DisparityPreviewWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit DisparityPreviewWidget( QWidget* parent = nullptr );

    ImageWidget *rectifyView() const;
    ImageWidget *disparityView() const;

protected:
    QPointer<ImageWidget> m_rectifyView;
    QPointer<ImageWidget> m_disparityView;

private:
    void initialize();

};

class PreviewWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit PreviewWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent = nullptr );
    ~PreviewWidget();

    BMControlWidget *bmControlWidget() const;

    void setDecimation( const VimbaDecimationType type );

    bool loadCalibrationFile( const std::string &fileName );

protected:
    QPointer< DisparityPreviewWidget > m_view;
    QPointer< DisparityControlWidget > m_controlWidget;
    QPointer< PCLViewer > m_3dWidget;

    AVT::VmbAPI::CameraPtr m_leftCamera;
    AVT::VmbAPI::CameraPtr m_rightCamera;

    virtual void timerEvent( QTimerEvent * ) override;

    StereoCalibrationData m_calibration;

    BMProcessor m_bmProcessor;
    GMProcessor m_gmProcessor;

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};

