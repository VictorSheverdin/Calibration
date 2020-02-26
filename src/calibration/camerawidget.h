#pragma once

#include <QSplitter>
#include <QMutex>

#include "src/common/templateprocessor.h"
#include "src/common/markerprocessor.h"

#include "parameterswidget.h"

#include "src/common/defs.h"

#include "src/common/vimbacamera.h"

#include <opencv2/opencv.hpp>

class CameraPreviewWidget;

class CameraWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CameraWidgetBase( QWidget* parent = nullptr );

    void setType(const TypeComboBox::Type type );
    void setCount( const cv::Size &count );
    void setTemplateSize( const double value );
    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );

    void setAdaptiveThreshold( const bool value );
    void setNormalizeImage( const bool value );
    void setFilterQuads( const bool value );
    void setFastCheck( const bool value );

    TypeComboBox::Type templateType() const;
    const cv::Size &templateCount() const;
    double templateSize() const;
    double intervalSize() const;
    bool resizeFlag() const;
    unsigned int frameMaximumSize() const;

    bool adaptiveThreshold() const;
    bool normalizeImage() const;
    bool filterQuads() const;
    bool fastCheck() const;

protected:
    TemplateProcessor m_previewTemplateProcessor;
    ArucoProcessor m_previewArucoProcessor;

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

public slots:
    void setSourceImage(const CvImage image);
    void setPreviewImage(const CvImage image);

protected slots:
    void updateFrame();

protected:
    QPointer< CameraPreviewWidget > m_previewWidget;

    MasterCamera m_camera;

    QMutex m_updateMutex;

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

    bool isTemplateExist() const;

    StereoFrame stereoFrame();

public slots:
    void setLeftSourceImage( const CvImage image );
    void setLeftDisplayedImage( const CvImage image );

    void setRightSourceImage( const CvImage image );
    void setRightDisplayedImage( const CvImage image );

protected slots:
    void updateFrame();

protected:
    QPointer<CameraPreviewWidget> m_leftCameraWidget;
    QPointer<CameraPreviewWidget> m_rightCameraWidget;

    StereoCamera m_camera;

    QMutex m_updateMutex;

private:
    void initialize();
};

template < int NUM >
class NCameraWidgetCore
{
public:
    NCameraWidgetCore() = default;

protected:
    QPointer< CameraPreviewWidget > m_cameraWidgets[ NUM ];
    cv::VideoCapture m_videoCaptures[ NUM ];
};

class TripleCameraWidget : public CameraWidgetBase, public NCameraWidgetCore< 3 >
{
    Q_OBJECT

public:
    TripleCameraWidget( const int camera1Index, const int camera2Index, const int camera3Index, QWidget* parent = nullptr );

    const CvImage sourceImage( const unsigned int cameraIndex ) const;
    const CvImage previewImage( const unsigned int cameraIndex ) const;

public slots:
    void setSource1Image( const unsigned int cameraIndex, const CvImage image );
    void setPreviewImage( const unsigned int cameraIndex, const CvImage image );

protected slots:
    void updatePreview();

protected:
    void updatePreview( const unsigned int cameraIndex );

private:
    void initialize( const int camera1Index, const int camera2Index, const int camera3Index );

};
