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
class GMControlWidget;
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

class DisparityWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    explicit DisparityWidgetBase( QWidget* parent = nullptr );

    BMControlWidget *bmControlWidget() const;
    GMControlWidget *gmControlWidget() const;

    bool loadCalibrationFile( const QString &fileName );

    void updateFrame( const CvImage leftFrame, const CvImage rightFrame );

signals:
    void valueChanged();

public slots:
    void loadCalibrationDialog();

protected:
    QPointer< DisparityPreviewWidget > m_view;
    QPointer< DisparityControlWidget > m_controlWidget;
    QPointer< PCLViewer > m_3dWidget;

    std::shared_ptr< BMDisparityProcessor > m_bmProcessor;
    std::shared_ptr< GMDisparityProcessor > m_gmProcessor;

    StereoProcessor m_processor;

private:
    void initialize();

};

class CameraDisparityWidget : public DisparityWidgetBase
{
    Q_OBJECT

public:
    explicit CameraDisparityWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

protected:
    VimbaCamera m_leftCam;
    VimbaCamera m_rightCam;

    void updateFrame();

    virtual void timerEvent( QTimerEvent * ) override;

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

    bool loadCalibrationFile( const QString &fileName );

    void addIcon( const CvImage &leftImage, const CvImage &rightImage );
    void insertIcon( const CvImage &leftImage, const CvImage &rightImage );

    void loadIcon( const QString &leftFileName, const QString &rightFileName );

public slots:
    void loadCalibrationDialog();
    void importDialog();

protected slots:
    void updateFrame();
    void updateFrame( DisparityIcon* icon );

protected:
    QPointer< DisparityWidgetBase > m_disparityWidget;
    QPointer< DisparityIconsWidget > m_iconsWidget;

private:
    void initialize();

};


