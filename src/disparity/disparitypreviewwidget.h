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
class DisparityResultIcon;
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

class ReconstructionViewWidget : public PCLWidget
{

public:
    ReconstructionViewWidget( QWidget* parent = nullptr );

private:
    void initialize();

    static void pickingEventHandler( const pcl::visualization::PointPickingEvent &event, void *viewer_void );

};

class ControlDisparityWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ControlDisparityWidget( QWidget* parent = nullptr );

    BMControlWidget *bmControlWidget() const;
    GMControlWidget *gmControlWidget() const;
    BMGPUControlWidget *bmGpuControlWidget() const;
    BPControlWidget *bpControlWidget() const;

    void loadCalibrationFile( const QString &fileName );

signals:
    void valueChanged();

public slots:
    void processFrame( const StampedStereoImage &frame );
    void processDisparity( const CvImage &color, const CvImage &disparity );

    void loadCalibrationDialog();

private slots:
    void updateFrame();

protected:
    QPointer< DisparityPreviewWidget > m_view;
    QPointer< DisparityControlWidget > m_controlWidget;
    QPointer< ReconstructionViewWidget > m_3dWidget;

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

class ViewDisparityWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ViewDisparityWidget( QWidget* parent = nullptr );

    void loadCalibrationFile( const QString &fileName );

public slots:
    void processDisparity( const CvImage &color, const CvImage &disparity );

    void loadCalibrationDialog();

protected:
    QPointer< DisparityPreviewWidget > m_view;
    QPointer< ReconstructionViewWidget > m_3dWidget;

    std::shared_ptr< StereoResultProcessor > m_processor;

private:
    void initialize();

};

class CameraDisparityWidget : public ControlDisparityWidget
{
    Q_OBJECT

public:
    explicit CameraDisparityWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

protected slots:
    void updateFrame();

protected:
    StereoCamera m_camera;

    QMutex m_updateMutex;

private:
    void initialize();

};

class DiskDisparityWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit DiskDisparityWidget( QWidget* parent = nullptr );

public slots:
    void clearIcons();

protected:
    QPointer< DisparityIconsWidget > m_iconsWidget;

    int m_iconCount;

    void dropIconCount();

private:
    void initialize();

};

class StereoDisparityWidget : public DiskDisparityWidget
{
    Q_OBJECT

public:
    explicit StereoDisparityWidget( QWidget* parent = nullptr );

    BMControlWidget *bmControlWidget() const;
    GMControlWidget *gmControlWidget() const;

    void loadCalibrationFile( const QString &fileName );
    void addStereoIcon( const QString &leftFileName, const QString &rightFileName );

public slots:
    void loadCalibrationDialog();
    void importStereoDialog();

protected slots:
    void updateFrame();
    void updateFrame( DisparityIcon* icon );

protected:
    QPointer< ControlDisparityWidget > m_disparityWidget;

private:
    void initialize();

};

class FileDisparityWidget : public DiskDisparityWidget
{
    Q_OBJECT

public:
    explicit FileDisparityWidget( QWidget* parent = nullptr );

    void loadCalibrationFile( const QString &fileName );
    void addDisparityIcon( const QString &colorFileName , const QString &disparityFileName);

public slots:
    void loadCalibrationDialog();
    void importDisparityDialog();

protected slots:
    void updateFrame();
    void updateFrame( DisparityResultIcon* icon );

protected:
    QPointer< ViewDisparityWidget > m_disparityWidget;

private:
    void initialize();

};


