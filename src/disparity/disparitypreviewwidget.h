#pragma once

#include <QWidget>
#include <QSplitter>

#include "src/common/defs.h"
#include "stereoprocessor.h"

#include "src/common/vimbacamera.h"

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

    BMControlWidget *bmControlWidget() const;

    bool loadCalibrationFile( const std::string &fileName );

protected:
    QPointer< DisparityPreviewWidget > m_view;
    QPointer< DisparityControlWidget > m_controlWidget;
    QPointer< PCLViewer > m_3dWidget;

    VimbaCamera m_leftCam;
    VimbaCamera m_rightCam;

    void updateFrame();

    std::shared_ptr<BMDisparityProcessor> m_bmProcessor;
    std::shared_ptr<GMDisparityProcessor> m_gmProcessor;

    StereoProcessor m_processor;

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};

