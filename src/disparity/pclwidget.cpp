#include "src/common/precompiled.h"

#include "pclwidget.h"

PCLViewer::PCLViewer( QWidget* parent )
    : QVTKWidget( parent )
{
    initialize();
}

void PCLViewer::initialize()
{
    m_pclViewer = std::unique_ptr< pcl::visualization::PCLVisualizer >( new pcl::visualization::PCLVisualizer( "PCLVisualizer", false ) );
    SetRenderWindow( m_pclViewer->getRenderWindow() );
    m_pclViewer->initCameraParameters ();

    m_pclViewer->setBackgroundColor( 127, 127, 127 );

    m_pclViewer->addPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB> ) );

    m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 );

}

void PCLViewer::setPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &cloud )
{
    m_pclViewer->updatePointCloud( cloud );
}
