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
    m_renderer = vtkSmartPointer<vtkRenderer>::New();

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

void PCLWidget::setPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud )
{
    if( !m_pclViewer->updatePointCloud( cloud ) ) {
        m_pclViewer->addPointCloud( cloud );
        m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 );
    }

    update();

}

void PCLWidget::update()
{
    m_pclViewer->getRenderWindow()->Render();

    QVTKOpenGLWidget::update();
}
