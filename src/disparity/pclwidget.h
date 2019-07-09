#pragma once

#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>

class PCLViewer : public QVTKWidget
{
    Q_OBJECT

public:
    PCLViewer( QWidget* parent = nullptr );

    void setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud );

public slots:
    void update();

protected:
    std::unique_ptr< pcl::visualization::PCLVisualizer > m_pclViewer;

private:
    void initialize();

    static void pickingEventHandler( const pcl::visualization::PointPickingEvent& event, void* viewer_void );

};
