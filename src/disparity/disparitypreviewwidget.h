#pragma once

#include <QWidget>
#include <QSplitter>

#include "src/common/defs.h"

#include "processorthread.h"
#include "elasprocessor.h"

#include "src/common/vimbacamera.h"

#include "src/common/pclwidget.h"
#include "src/common/calibrationdatabase.h"

class ImageWidget;
class DisparityControlWidget;
class BMControlWidget;
class BMGPUControlWidget;
class GMControlWidget;
class BPControlWidget;
class DisparityIcon;
class DisparityIconsWidget;

class DisparityPreviewWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit DisparityPreviewWidget( QWidget* parent = nullptr );

    ImageWidget *rectifyView() const;
    ImageWidget *disparityView() const;

protected:
    QPointer< ImageWidget > m_rectifyView;
    QPointer< ImageWidget > m_disparityView;

private:
    void initialize();

};

class View3DWidget : public PCLWidget
{

public:
    View3DWidget( QWidget* parent = nullptr );

private:
    void initialize();

    static void pickingEventHandler( const pcl::visualization::PointPickingEvent &event, void *viewer_void );

};

class DisparityWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    explicit DisparityWidgetBase( QWidget* parent = nullptr );

    BMControlWidget *bmControlWidget() const;
    GMControlWidget *gmControlWidget() const;
    BMGPUControlWidget *bmGpuControlWidget() const;
    BPControlWidget *bpControlWidget() const;

    void loadCalibrationFile( const QString &fileName );

signals:
    void valueChanged();

public slots:
    void processFrame( const StereoFrame &frame );

    void loadCalibrationDialog();

private slots:
    void updateFrame();

protected:
    QPointer< DisparityPreviewWidget > m_view;
    QPointer< DisparityControlWidget > m_controlWidget;
    QPointer< View3DWidget > m_3dWidget;

    QMutex m_updateMutex;

    std::shared_ptr< BMDisparityProcessor > m_bmProcessor;
    std::shared_ptr< GMDisparityProcessor > m_gmProcessor;
    std::shared_ptr< BMGPUDisparityProcessor > m_bmGpuProcessor;
    std::shared_ptr< BPDisparityProcessor > m_bpProcessor;
    std::shared_ptr< CSBPDisparityProcessor > m_csbpProcessor;
    std::shared_ptr< ElasDisparityProcessor > m_elasProcessor;

    std::shared_ptr< StereoResultProcessor > m_processor;

    ProcessorThread m_processorThread;

    // std::chrono::time_point< std::chrono::system_clock > m_time;

private:
    void initialize();

};

class CameraDisparityWidget : public DisparityWidgetBase
{
    Q_OBJECT

public:
    explicit CameraDisparityWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

protected:
    StereoCamera m_camera;

    QMutex m_updateMutex;

    void updateFrame();

private:
    void initialize();

};

class ImageDisparityWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ImageDisparityWidget( QWidget* parent = nullptr );

    BMControlWidget *bmControlWidget() const;
    GMControlWidget *gmControlWidget() const;

    void loadCalibrationFile( const QString &fileName );
    void addIcon( const QString &leftFileName, const QString &rightFileName );

    int m_iconCount;

public slots:
    void loadCalibrationDialog();
    void importDialog();

    void clearIcons();

protected slots:
    void updateFrame();
    void updateFrame( DisparityIcon* icon );

protected:
    QPointer< DisparityWidgetBase > m_disparityWidget;
    QPointer< DisparityIconsWidget > m_iconsWidget;

private:
    void initialize();

    void dropIconCount();

};


