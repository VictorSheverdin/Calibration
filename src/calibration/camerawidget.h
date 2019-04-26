#pragma once

#include <QThread>
#include <QSplitter>

#include "templateprocessor.h"

#include "src/common/defs.h"

#include <opencv2/opencv.hpp>

#include "VimbaCPP/Include/VimbaCPP.h"

class PreviewWidget;

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

    virtual void setDecimation( const VimbaDecimationType type ) = 0;

protected:
    TemplateProcessor m_processor;

    static const int m_aquireInterval = 30;
    static const int m_aquireCount = 10;

    int m_timerId;

private:
    void initialize();
};

class MonocularFrameObserver : public AVT::VmbAPI::IFrameObserver
{
public :
    MonocularFrameObserver( AVT::VmbAPI::CameraPtr pCamera );

    virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame ) override;
};

class MonocularCameraWidget : public CameraWidgetBase
{
    Q_OBJECT

public:
    MonocularCameraWidget( const std::string &cameraIp, QWidget* parent = nullptr );
    ~MonocularCameraWidget();

    const CvImage sourceImage() const;
    const CvImage previewImage() const;
    const std::vector<cv::Point2f> &previewPoints() const;

    bool isTemplateExist() const;

    virtual void setDecimation( const VimbaDecimationType type ) override;

public slots:
    void setSourceImage(const CvImage image);
    void setPreviewImage(const CvImage image);
    void setPreviewPoints(const std::vector<cv::Point2f> &points );

protected slots:
    void updatePreview();

protected:
    QPointer<PreviewWidget> m_previewWidget;

    AVT::VmbAPI::CameraPtr m_camera;

    virtual void timerEvent(QTimerEvent *event) override;

private:
    void initialize( const std::string &cameraIp );

};

class StereoCameraWidget : public CameraWidgetBase
{
    Q_OBJECT

public:
    StereoCameraWidget( const std::string &cameraIp, const std::string &rightCameraIp, QWidget* parent = nullptr );
    ~StereoCameraWidget();

    const CvImage leftSourceImage() const;
    const CvImage leftDisplayedImage() const;
    const std::vector<cv::Point2f> &leftPreviewPoints() const;

    const CvImage rightSourceImage() const;
    const CvImage rightDisplayedImage() const;
    const std::vector<cv::Point2f> &rightPreviewPoints() const;

    static CvImage makeOverlappedPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );
    static CvImage makeStraightPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage );

    bool isTemplateExist() const;

    virtual void setDecimation( const VimbaDecimationType type ) override;

public slots:
    void setLeftSourceImage( const CvImage image );
    void setLeftDisplayedImage( const CvImage image );
    void setLeftPreviewPoints(const std::vector<cv::Point2f> &points );

    void setRightSourceImage( const CvImage image );
    void setRightDisplayedImage( const CvImage image );
    void setRightPreviewPoints( const std::vector<cv::Point2f> &points );

protected slots:
    void updatePreview();

protected:
    QPointer<PreviewWidget> m_leftCameraWidget;
    QPointer<PreviewWidget> m_rightCameraWidget;

    AVT::VmbAPI::CameraPtr m_leftCamera;
    AVT::VmbAPI::CameraPtr m_rightCamera;

    virtual void timerEvent( QTimerEvent * ) override;

    void updateLeftPreview();
    void updateRightPreview();

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );
};

template < int NUM >
class NCameraWidgetCore
{
public:
    NCameraWidgetCore() = default;

protected:
    QPointer< PreviewWidget > m_cameraWidgets[ NUM ];
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
