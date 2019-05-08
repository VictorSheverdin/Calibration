#include "src/common/precompiled.h"

#include "pclwidget.h"

PCLViewer::PCLViewer( QWidget* parent )
    : QVTKWidget( parent )
{
    initialize();
}

void PCLViewer::initialize()
{
    pclviewer = std::unique_ptr< pcl::visualization::PCLVisualizer >( new pcl::visualization::PCLVisualizer( "PCLVisualizer", false ) );
    SetRenderWindow( pclviewer->getRenderWindow() );
    pclviewer->initCameraParameters ();

    pclviewer->addPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB> ), "disparity" );

    pclviewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "disparity" );

}
