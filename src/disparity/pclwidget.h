#pragma once

#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>

class PCLViewer : public QVTKWidget
{
    Q_OBJECT

public:
    PCLViewer( QWidget* parent = nullptr );

    std::unique_ptr< pcl::visualization::PCLVisualizer > pclviewer;

private:
    void initialize();

};
