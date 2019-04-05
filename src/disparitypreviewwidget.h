#pragma once

#include <QWidget>
#include <QSplitter>

#include <opencv2/opencv.hpp>

class ImageWidget;
class DisparityControlWidget;
class QVTKWidget;

class PreviewWidget : public QSplitter
{
    Q_OBJECT

public:
    QSplitter( QWidget* parent = nullptr );
};

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
    int disp12MaxDiff() const;
    int smallerBlockSize() const;
    int filterLambda() const;
    int lrcThresh() const;

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
    void setDisp12MaxDiff( const int value );
    void setSmallerBlockSize( const int value );
    void setFilterLambda( const int value );
    void setLrcThresh( const int value ) const;

protected:
    QPointer<ImageWidget> m_rectifyView;
    QPointer<ImageWidget> m_disparityView;
    QPointer<ImageWidget> m_filteredDisparityView;
    QPointer<DisparityControlWidget> m_controlWidget;
    QPointer<QVTKWidget> m_3dViewWidget;

    cv::VideoCapture m_leftCapture;
    cv::VideoCapture m_rightCapture;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );

};
