#pragma once

#include <QWidget>
#include <QSplitter>

#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>

class ImageWidget;
class DisparityControlWidget;
class QVTKWidget;

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

    cv::VideoCapture m_leftCapture;
    cv::VideoCapture m_rightCapture;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );

};

class PreviewWidget : public QSplitter
{
    Q_OBJECT

public:
    PreviewWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent = nullptr );

protected:
    DisparityPreviewWidget *m_disparityWidget;
    QVTKWidget *m_3dWidget;

    pcl::visualization::PCLVisualizer *m_visualizer;

private:
    void initialize( const int leftCameraIndex, const int rightCameraIndex );

};
