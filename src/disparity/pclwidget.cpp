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

//    pclviewer->setBackgroundColor( 32.0/256.0, 162.0/256.0, 186.0/256.0 );

    pclviewer->addPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB> ), "disparity" );

    pclviewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "disparity" );

}
