#pragma once

#include <QWidget>

#include <opencv2/opencv.hpp>

class ImageWidget;
class DisparityControlWidget;

class DisparityPreviewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DisparityPreviewWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent = nullptr );

    int prefilterSize() const;
    int prefilterCap() const;
    int sadWindowSize() const;
    int minDisparity() const;
    int numDisparities() const;
    int textureThreshold() const;
    int uniquessRatio() const;
    int speckleWindowSize() const;
    int speckleRange() const;

public slots:
    void setPrefilterSize( const int value );
    void setPrefilterCap( const int value );
    void setSadWindowSize( const int value );
    void setMinDisparity( const int value );
    void setNumDisparities( const int value );
    void setTextureThreshold( const int value );
    void setUniquessRatio( const int value );
    void setSpeckleWindowSize( const int value );
    void setSpeckleRange( const int value );

protected:
    QPointer<ImageWidget> m_leftView;
    QPointer<ImageWidget> m_rightView;
    QPointer<ImageWidget> m_disparityView;
    QPointer<ImageWidget> m_filteredDisparityView;
    QPointer<DisparityControlWidget> m_controlWidget;

    cv::VideoCapture m_leftCapture;
    cv::VideoCapture m_rightCapture;

    virtual void timerEvent(QTimerEvent *event) override;

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );

};
