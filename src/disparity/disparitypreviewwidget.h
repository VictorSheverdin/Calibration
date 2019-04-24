#pragma once

#include <QWidget>
#include <QSplitter>

#include "src/common/defs.h"

#include <opencv2/opencv.hpp>

#include "VimbaCPP/Include/VimbaCPP.h"

#include <pcl/visualization/cloud_viewer.h>

#include "src/common/calibrationdata.h"

class ImageWidget;
class DisparityControlWidget;
class QVTKWidget;

class DisparityPreviewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DisparityPreviewWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent = nullptr );

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

    void setDecimation( const VimbaDecimationType type );

    bool loadCalibrationFile( const std::string &fileName );

protected:
    QPointer<ImageWidget> m_rectifyView;
    QPointer<ImageWidget> m_disparityView;
    QPointer<ImageWidget> m_filteredDisparityView;
    QPointer<DisparityControlWidget> m_controlWidget;

    AVT::VmbAPI::CameraPtr m_leftCamera;
    AVT::VmbAPI::CameraPtr m_rightCamera;

    virtual void timerEvent( QTimerEvent * ) override;

    StereoCalibrationData m_calibration;

    cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};

class PreviewWidget : public QSplitter
{
    Q_OBJECT

public:
    PreviewWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent = nullptr );

    bool loadCalibrationFile( const std::string &fileName );

protected:
    DisparityPreviewWidget *m_disparityWidget;
    QVTKWidget *m_3dWidget;

    pcl::visualization::PCLVisualizer *m_visualizer;

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};
