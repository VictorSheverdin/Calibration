#pragma once

#include "colorpoint.h"

#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>

class PCLWidget : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    PCLWidget( QWidget* parent = nullptr );

    void setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &cloud, const std::string &id = "cloud" );

    void setPointCloud( const std::vector< ColorPoint3d > &points,
                        const std::string &id = "cloud",
                        const Eigen::Vector4f &origin = Eigen::Vector4f::Zero(),
                        const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity() );

    void setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation );

    bool contains( const std::string &id ) const ;

    void showPointCloud( const std::string &id, const bool flag ) ;

    void exportPointCloud( const std::string &fileName );

public slots:
    void update();

protected:
    vtkSmartPointer< vtkRenderer > m_renderer;

    pcl::visualization::PCLVisualizer::Ptr m_pclViewer;

private:
    void initialize();

};
