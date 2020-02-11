#include "precompiled.h"

#include "pclwidget.h"

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPointPicker.h>

PCLWidget::PCLWidget( QWidget* parent )
    : QVTKOpenGLWidget( parent )
{
    initialize();
}

void PCLWidget::initialize()
{
    m_renderer = vtkSmartPointer< vtkRenderer >::New();

    m_renderer->SetBackground( 127, 127, 127 );
    m_renderer->SetBackground2( 1, 1, 1 );
    m_renderer->SetGradientBackground( true );

    vtkNew< vtkGenericOpenGLRenderWindow > renderWindow;
    renderWindow->AddRenderer( m_renderer.Get() );

    m_pclViewer = pcl::visualization::PCLVisualizer::Ptr( new pcl::visualization::PCLVisualizer( m_renderer.Get(), renderWindow.Get(), "PCLVisualizer", false ) );
    SetRenderWindow( m_pclViewer->getRenderWindow() );

    m_pclViewer->initCameraParameters();

    m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 );

    m_pclViewer->setCameraPosition( 0, 0, -10, 0, -1, 0 );

    m_pclViewer->addCoordinateSystem( 0.5 );

    m_pclViewer->setShowFPS( true );

}

void PCLWidget::setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const std::string &id )
{
    if( !m_pclViewer->updatePointCloud( cloud, id ) ) {
        m_pclViewer->addPointCloud( cloud, id );
        m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id );
    }

    update();

}

void PCLWidget::setPointCloud( const std::list< ColorPoint3d > &points, const std::string &id )
{
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );

    for ( auto &i : points ) {

        auto point = i.point();

        pcl::PointXYZRGB pclPoint;
        pclPoint.x = point.x;
        pclPoint.y = point.y;
        pclPoint.z = point.z;

        auto color = i.color();

        pclPoint.r = std::max( 0.0, std::min( color[ 2 ], 255.0 ) );
        pclPoint.g = std::max( 0.0, std::min( color[ 1 ], 255.0 ) );
        pclPoint.b = std::max( 0.0, std::min( color[ 0 ], 255.0 ) );
        pclPoint.a = 255;

        pointCloud->push_back( pclPoint );

    }

    setPointCloud( pointCloud, id );

}

bool PCLWidget::contains( const std::string &id ) const
{
    return m_pclViewer->contains( id );
}

void PCLWidget::update()
{
    m_pclViewer->getRenderWindow()->Render();

    QVTKOpenGLWidget::update();
}
