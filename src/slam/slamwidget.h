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

protected:
    QPointer< ImageWidget > m_pointsWidget;
    QPointer< ImageWidget > m_tracksWidget;

private:
    void initialize();

};

class PCLWidget : public QVTKOpenGLWidget
{
    Q_OBJECT

public:
    PCLWidget( QWidget* parent = nullptr );

    void setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud );

public slots:
    void update();

protected:
    pcl::visualization::PCLVisualizer::Ptr m_pclViewer;

private:
    void initialize();

    static void pickingEventHandler( const pcl::visualization::PointPickingEvent& event, void* viewer_void );

};

class SlamThread;

class SlamWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit SlamWidget( QWidget* parent = nullptr );
    ~SlamWidget();

public slots:
    void setGeometry( const SlamGeometry &geo );
    void updateViews();

protected:
    QPointer< ImagesWidget > m_imagesWidget;
    QPointer< PCLWidget > m_pclWidget;

    QPointer< SlamThread > m_slamThread;

private:
    void initialize();

};
