#pragma once

#include <QSplitter>
#include <QMutex>

#include "src/common/templateprocessor.h"
#include "src/common/markerprocessor.h"

#include "parameterswidget.h"

#include "src/common/defs.h"

#include "src/common/vimbacamera.h"

#include <opencv2/opencv.hpp>

#include "threads.h"

class CameraPreviewWidget;

class CameraWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CameraWidgetBase( QWidget* parent = nullptr );

    void setType(const TypeComboBox::Type type );
    TypeComboBox::Type type() const;

protected:
    TypeComboBox::Type m_type;

private:
    void initialize();
};

class MonocularCameraWidget : public CameraWidgetBase
{
    Q_OBJECT

public:
    MonocularCameraWidget( const QString &cameraIp, QWidget* parent = nullptr );

    const CvImage sourceImage() const;
    const CvImage previewImage() const;

    void setTemplateExist( const bool value );
    bool isTemplateExist() const;

    const TemplateProcessor &templateProcessor() const;
    TemplateProcessor &templateProcessor();

    const ArucoProcessor &markerProcessor() const;
    ArucoProcessor &markerProcessor();

    void setType(const TypeComboBox::Type type );
    void setCount( const cv::Size &count );
    void setTemplateSize( const double value );
    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );

    void setAdaptiveThreshold( const bool value );
    void setNormalizeImage( const bool value );
    void setFilterQuads( const bool value );
    void setFastCheck( const bool value );

public slots:
    void setSourceImage(const CvImage image);
    void setPreviewImage(const CvImage image);

protected slots:
    void reciveFrame();
    void updateView();

protected:
    QPointer< CameraPreviewWidget > m_previewWidget;

    MasterCamera m_camera;

    MonocularProcessorThread m_previewThread;

    mutable QMutex m_updateMutex;

private:
    void initialize();

};

class StereoCameraWidget : public CameraWidgetBase
{
    Q_OBJECT

public:
    StereoCameraWidget( const QString &cameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

    const CvImage leftSourceImage() const;
    const CvImage leftDisplayedImage() const;

    const CvImage rightSourceImage() const;
    const CvImage rightDisplayedImage() const;

    const TemplateProcessor &templateProcessor() const;
    TemplateProcessor &templateProcessor();

    const ArucoProcessor &markerProcessor() const;
    ArucoProcessor &markerProcessor();

    void setType(const TypeComboBox::Type type );
    void setCount( const cv::Size &count );
    void setTemplateSize( const double value );
    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );

    void setAdaptiveThreshold( const bool value );
    void setNormalizeImage( const bool value );
    void setFilterQuads( const bool value );
    void setFastCheck( const bool value );

    TypeComboBox::Type type() const;
    const cv::Size &templateCount() const;
    double templateSize() const;
    double intervalSize() const;
    bool resizeFlag() const;
    unsigned int frameMaximumSize() const;

    bool adaptiveThreshold() const;
    bool normalizeImage() const;
    bool filterQuads() const;
    bool fastCheck() const;

    bool isTemplateExist() const;

    StereoFrame sourceFrame();

public slots:
    void setLeftSourceImage( const CvImage image );
    void setLeftDisplayedImage( const CvImage image );

    void setRightSourceImage( const CvImage image );
    void setRightDisplayedImage( const CvImage image );

protected slots:
    void reciveFrame();
    void updateView();

protected:
    QPointer<CameraPreviewWidget> m_leftCameraWidget;
    QPointer<CameraPreviewWidget> m_rightCameraWidget;

    StereoProcessorThread m_previewThread;

    StereoCamera m_camera;

    StereoFrame m_sourceFrame;

    QMutex m_updateMutex;

private:
    void initialize();
};
