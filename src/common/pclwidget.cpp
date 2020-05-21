#include "precompiled.h"

#include "pclwidget.h"

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointPicker.h>

#include <pcl/io/pcd_io.h>

PCLWidget::PCLWidget( QWidget* parent )
    : QVTKOpenGLNativeWidget( parent )
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
    m_pclViewer->setupInteractor( GetInteractor(), GetRenderWindow() );

    m_pclViewer->initCameraParameters();

    m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 );

    m_pclViewer->setCameraPosition( 0, 0, -10, 0, -1, 0 );
    m_pclViewer->setCameraClipDistances( 0.1, 10000 );

    m_pclViewer->setShowFPS( true );

    m_pclViewer->getInteractorStyle()->SetAutoAdjustCameraClippingRange( false );

    /*vtkSmartPointer< vtkInteractorStyleTrackballCamera > style = vtkSmartPointer< vtkInteractorStyleTrackballCamera >::New();

    style->SetAutoAdjustCameraClippingRange( false );

    renderWindow->GetInteractor()->SetInteractorStyle( style );*/

}

void PCLWidget::setPointCloud( const pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const std::string &id )
{
    if( !m_pclViewer->updatePointCloud( cloud, id ) ) {
        m_pclViewer->addPointCloud( cloud, id );
        m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id );
    }

    update();

}

void PCLWidget::setPointCloud( const std::list< ColorPoint3d > &points, const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
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

    pointCloud->sensor_origin_ = origin;
    pointCloud->sensor_orientation_ = orientation;

    setPointCloud( pointCloud, id );

}

void PCLWidget::setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
{
    Eigen::Affine3f pose;

    pose.setIdentity();

    Eigen::Vector3f translation( origin[ 0 ], origin[ 1 ], origin[ 2 ] );
    pose.translate( translation );

    pose.rotate( orientation );

    m_pclViewer->updatePointCloudPose( id, pose );
}

bool PCLWidget::contains( const std::string &id ) const
{
    return m_pclViewer->contains( id );
}

void PCLWidget::showPointCloud( const std::string &id, const bool flag )
{
    m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, flag ? 1.0 : 0.0, id );
}

void PCLWidget::update()
{
    m_pclViewer->getRenderWindow()->Render();

    QVTKOpenGLNativeWidget::update();
}

