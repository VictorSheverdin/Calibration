#pragma once

#include <QSplitter>

#include <QVTKOpenGLWidget.h>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "slamgeometry.h"

#include "src/common/imagewidget.h"
#include "src/common/pclwidget.h"

class ImagesWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ImagesWidget( QWidget* parent = nullptr );

public slots:
    void setPointsImage( const CvImage &image );
    void setTracksImage( const CvImage &image );
    void setStereoImage( const CvImage &image );

protected:
    QPointer< ImageWidget > m_pointsWidget;
    QPointer< ImageWidget > m_tracksWidget;
    QPointer< ImageWidget > m_stereoWidget;

private:
    void initialize();

};

class View3dWidget : public PCLWidget
{
    Q_OBJECT

public:
    View3dWidget( QWidget* parent = nullptr );

    void setPath( const std::list< StereoCameraMatrix > &path );

protected:
    vtkSmartPointer< vtkActor > m_leftTrajectoryActor;
    vtkSmartPointer< vtkActor > m_rightTrajectoryActor;

    vtkSmartPointer< vtkActor > m_leftCameraActor;
    vtkSmartPointer< vtkActor > m_rightCameraActor;

    void setLeftPath( std::list< cv::Vec3d > &points );
    void setRightPath( std::list< cv::Vec3d > &points );

    void setFrustum( const StereoCameraMatrix &cameraMatrix );
    void setLeftFrustum( const ProjectionMatrix &cameraMatrix );
    void setRightFrustum( const ProjectionMatrix &cameraMatrix );

private:
    void initialize();

};

class SlamThread;

class SlamWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit SlamWidget( QWidget* parent = nullptr );
    ~SlamWidget();

    void setPath( const std::list< StereoCameraMatrix > &path );
    void setSparseCloud( const std::list< ColorPoint3d > &points );
    void setPointCloud( const std::list< ColorPoint3d > &points, const std::string &id,
                        const Eigen::Vector4f &origin = Eigen::Vector4f::Zero(),
                        const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity() );
    void setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation );

public slots:
    void updateViews();
    void updateImages();
    void update3dView();

protected slots:
    void updatePath();
    void updateSparseCloud();
    void updateDensePointCloud();

protected:
    QPointer< ImagesWidget > m_imagesWidget;
    QPointer< View3dWidget > m_pclWidget;

    QPointer< SlamThread > m_slamThread;

    std::list< StereoCameraMatrix > path() const;
    std::list< ColorPoint3d > sparseCloud() const;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize();

};
