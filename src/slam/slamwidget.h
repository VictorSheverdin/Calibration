#pragma once

#include <QSplitter>

#include <QVTKOpenGLWidget.h>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "slamgeometry.h"

#include "src/common/imagewidget.h"

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

class PCLWidget : public QVTKOpenGLWidget
{
    Q_OBJECT

public:
    PCLWidget( QWidget* parent = nullptr );

    void setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud );
    void setLeftPath( const pcl::PointCloud< pcl::PointXYZ >::Ptr path );
    void setRightPath( const pcl::PointCloud< pcl::PointXYZ >::Ptr path );

public slots:
    void update();

protected:
    vtkSmartPointer<vtkRenderer> m_renderer;

    pcl::visualization::PCLVisualizer::Ptr m_pclViewer;

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

public slots:
    void updateViews();

protected:
    QPointer< ImagesWidget > m_imagesWidget;
    QPointer< PCLWidget > m_pclWidget;

    QPointer< SlamThread > m_slamThread;

    void setGeometry( const SlamGeometry &geo );

private:
    void initialize();

};
