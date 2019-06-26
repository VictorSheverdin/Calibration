#include "src/common/precompiled.h"

#include "pclwidget.h"

#include <vtkPointPicker.h>

#include "application.h"

PCLViewer::PCLViewer( QWidget* parent )
    : QVTKWidget( parent )
{
    initialize();
}

void PCLViewer::pickingEventHandler( const pcl::visualization::PointPickingEvent& event, void* viewer_void )
{
    float x, y, z;

    if (event.getPointIndex () == -1) {
        return;
    }

    event.getPoint( x, y, z );

    auto distance = sqrt( x * x + y * y + z * z ) * 15.7;

    std::stringstream ss;
    ss << distance;

/*    application()->setStatusBarText( tr( "Coordinates: x = %1, y = %2, z = %3, Distance = %4" )
                                     .arg( QString::number( x ) ).arg( QString::number( y ) ).arg( QString::number( z ) ).arg( QString::number( distance ) ) ); */

    application()->setStatusBarText( tr( "Distance = %1" )
                                     .arg( QString::number( distance ) ) );

    auto widget = reinterpret_cast< pcl::visualization::PCLVisualizer *>( viewer_void );

    widget->removeText3D( "Text" );
    widget->addText3D( ss.str(), pcl::PointXYZ( x, y, z - 0.1 ), 0.2, 1.0, 0, 0, "Text" );

}

void PCLViewer::initialize()
{
    m_pclViewer = std::unique_ptr< pcl::visualization::PCLVisualizer >( new pcl::visualization::PCLVisualizer( "PCLVisualizer" ) );
    SetRenderWindow( m_pclViewer->getRenderWindow() );
    m_pclViewer->initCameraParameters ();

    m_pclViewer->setBackgroundColor( 127, 127, 127 );

    m_pclViewer->addPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB> ) );

    m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 );

    // m_pclViewer->addOrientationMarkerWidgetAxes( GetInteractor() );
    // m_pclViewer->addCoordinateSystem( 0.1 );
    m_pclViewer->registerPointPickingCallback( PCLViewer::pickingEventHandler, m_pclViewer.get() );

    m_pclViewer->setCameraPosition( 0, 0, -1, 0, 1, 0 );

}

void PCLViewer::setPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr &cloud )
{
    m_pclViewer->updatePointCloud( cloud );
}
