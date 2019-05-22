#pragma once

#include <QSplitter>
#include <QMutex>

#include "templateprocessor.h"

#include "src/common/defs.h"

#include "src/common/vimbacamera.h"

#include <opencv2/opencv.hpp>

class CameraPreviewWidget;

class CameraWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CameraWidgetBase( QWidget* parent = nullptr );

    void setType(const TemplateProcessor::Type type );
    void setCount( const cv::Size &count );
    void setTemplateSize( const double value );
    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );

    void setAdaptiveThreshold( const bool value );
    void setNormalizeImage( const bool value );
    void setFilterQuads( const bool value );
    void setFastCheck( const bool value );

    TemplateProcessor::Type templateType() const;
    const cv::Size &templateCount() const;
    double templateSize() const;
    bool resizeFlag() const;
    unsigned int frameMaximumFlag() const;

    bool adaptiveThreshold() const;
    bool normalizeImage() const;
    bool filterQuads() const;
    bool fastCheck() const;

protected:
    TemplateProcessor m_processor;

    static const int m_aquireInterval = 30;
    static const int m_aquireCount = 10;

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
    const std::vector<cv::Point2f> &previewPoints() const;

    bool isTemplateExist() const;

public slots:
    void setSourceImage(const CvImage image);
    void setPreviewImage(const CvImage image);
    void setPreviewPoints(const std::vector<cv::Point2f> &points );

protected slots:
    void updateFrame();

protected:
    QPointer< CameraPreviewWidget > m_previewWidget;

    VimbaCamera m_camera;

    virtual void timerEvent( QTimerEvent * ) override;

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
    const std::vector<cv::Point2f> &leftPreviewPoints() const;

    const CvImage rightSourceImage() const;
    const CvImage rightDisplayedImage() const;
    const std::vector<cv::Point2f> &rightPreviewPoints() const;

    bool isTemplateExist() const;

public slots:
    void setLeftSourceImage( const CvImage image );
    void setLeftDisplayedImage( const CvImage image );
    void setLeftPreviewPoints(const std::vector<cv::Point2f> &points );

    void setRightSourceImage( const CvImage image );
    void setRightDisplayedImage( const CvImage image );
    void setRightPreviewPoints( const std::vector<cv::Point2f> &points );

protected slots:
    void updateLeftFrame();
    void updateRightFrame();

protected:
    QPointer<CameraPreviewWidget> m_leftCameraWidget;
    QPointer<CameraPreviewWidget> m_rightCameraWidget;

    VimbaCamera m_leftCamera;
    VimbaCamera m_rightCamera;

    QMutex m_leftUpdateMutex;
    QMutex m_rightUpdateMutex;

    virtual void timerEvent( QTimerEvent * ) override;

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

    static CvImage createPreview( const CvImage &previewImage1, const CvImage &previewImage2, const CvImage &previewImage3 );

public slots:
    void setSource1Image( const unsigned int cameraIndex, const CvImage image );
    void setPreviewImage( const unsigned int cameraIndex, const CvImage image );

protected slots:
    void updatePreview();

protected:
    virtual void timerEvent( QTimerEvent *event ) override;

    void updatePreview( const unsigned int cameraIndex );

private:
    void initialize( const int camera1Index, const int camera2Index, const int camera3Index );

};
