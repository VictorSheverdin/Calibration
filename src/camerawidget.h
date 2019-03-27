#pragma once

#include <QThread>
#include <QSplitter>

#include "templateprocessor.h"

#include <opencv2/opencv.hpp>

class PreviewWidget;

class CameraWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CameraWidgetBase( QWidget* parent = nullptr );

    void setType(const TemplateProcessor::Type type );
    void setCount( const cv::Size &count );
    void setSize( const double value );
    void setResizeFlag( const bool value );
    void setFrameMaximumSize( const unsigned int value );

    TemplateProcessor::Type type() const;
    const cv::Size &count() const;
    double size() const;
    bool resizeFlag() const;
    unsigned int frameMaximumFlag() const;

protected:
    TemplateProcessor m_processor;

private:
    void initialize();
};

class MonocularCameraWidget : public CameraWidgetBase
{
    Q_OBJECT

public:
    MonocularCameraWidget( const int cameraIndex, QWidget* parent = nullptr );

    const CvImage sourceImage() const;
    const CvImage previewImage() const;
    const std::vector<cv::Point2f> &previewPoints() const;

public slots:
    void setSourceImage(const CvImage image);
    void setPreviewImage(const CvImage image);
    void setPreviewPoints( const std::vector<cv::Point2f> &points );

protected slots:
    void updatePreview();

protected:
    QPointer<PreviewWidget> m_previewWidget;

    cv::VideoCapture m_capture;

    virtual void timerEvent(QTimerEvent *event) override;



private:
    void initialize( const int cameraIndex );

};

class StereoCameraWidget : public CameraWidgetBase
{
    Q_OBJECT

public:
    StereoCameraWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent = nullptr );

    const CvImage leftSourceImage() const;
    const CvImage leftDisplayedImage() const;
    const std::vector<cv::Point2f> &leftPreviewPoints() const;

    const CvImage rightSourceImage() const;
    const CvImage rightDisplayedImage() const;
    const std::vector<cv::Point2f> &rightPreviewPoints() const;

    static CvImage createPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );

public slots:
    void setLeftSourceImage( const CvImage image );
    void setLeftDisplayedImage( const CvImage image );
    void setLeftPreviewPoints( const std::vector<cv::Point2f> &points );

    void setRightSourceImage( const CvImage image );
    void setRightDisplayedImage( const CvImage image );
    void setRightPreviewPoints( const std::vector<cv::Point2f> &points );

protected slots:
    void updatePreview();

protected:
    QPointer<PreviewWidget> m_leftCameraWidget;
    QPointer<PreviewWidget> m_rightCameraWidget;

    cv::VideoCapture m_leftCapture;
    cv::VideoCapture m_rightCapture;

    virtual void timerEvent( QTimerEvent *event ) override;

    void updateLeftPreview();
    void updateRightPreview();

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );
};

template <int NUM>
class NCameraWidgetCore
{
public:
    NCameraWidgetCore() = default;

protected:
    QPointer<PreviewWidget> m_cameraWidgets[NUM];
    cv::VideoCapture m_videoCaptures[NUM];
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
