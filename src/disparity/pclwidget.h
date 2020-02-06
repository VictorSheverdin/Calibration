#pragma once

#include <QVTKOpenGLWidget.h>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>

class PCLViewer : public QVTKOpenGLWidget
{
    Q_OBJECT

public:
    PCLViewer( QWidget* parent = nullptr );

    void setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud );

public slots:
    void update();

protected:
    vtkSmartPointer< vtkRenderer > m_renderer;

    pcl::visualization::PCLVisualizer::Ptr m_pclViewer;

private:
    void initialize();

};
