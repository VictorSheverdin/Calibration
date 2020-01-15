#include "src/common/precompiled.h"

#include "slamwidget.h"

#include "slamthread.h"
#include <vtkPointPicker.h>

#include <vtkGenericOpenGLRenderWindow.h>

// ImagesWidget
ImagesWidget::ImagesWidget( QWidget* parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void ImagesWidget::initialize()
{
    m_pointsWidget = new ImageWidget( this );
    addWidget( m_pointsWidget );

    m_tracksWidget = new ImageWidget( this );
    addWidget( m_tracksWidget );

}

void ImagesWidget::setPointsImage( const CvImage &image )
{
    m_pointsWidget->setImage( image );
}

void ImagesWidget::setTracksImage( const CvImage &image )
{
    m_tracksWidget->setImage( image );
}

// PCLViewer
PCLWidget::PCLWidget( QWidget* parent )
    : QVTKOpenGLWidget( parent )
{
    initialize();
}

void PCLWidget::pickingEventHandler( const pcl::visualization::PointPickingEvent& event, void* viewer_void )
{
    float x, y, z;

    if (event.getPointIndex () == -1) {
        return;
    }

    event.getPoint( x, y, z );

    auto distance = sqrt( x * x + y * y + z * z ) * 15.7;

    std::stringstream ss;
    ss << distance;

    auto widget = reinterpret_cast< pcl::visualization::PCLVisualizer * >( viewer_void );

    widget->removeText3D( "Text" );
    widget->addText3D( ss.str(), pcl::PointXYZ( x, y, z - 0.1 ), 0.2, 1.0, 0, 0, "Text" );

}

void PCLWidget::initialize()
{
    vtkNew< vtkRenderer > renderer;

    renderer->SetBackground( 127, 127, 127 );
    renderer->SetBackground2( 1, 1, 1 );
    renderer->SetGradientBackground( true );

    vtkNew< vtkGenericOpenGLRenderWindow > renderWindow;
    renderWindow->AddRenderer( renderer.Get() );

    m_pclViewer = pcl::visualization::PCLVisualizer::Ptr( new pcl::visualization::PCLVisualizer( renderer.Get(), renderWindow.Get(), "PCLVisualizer", false ) );
    SetRenderWindow( m_pclViewer->getRenderWindow() );

    m_pclViewer->initCameraParameters();

    m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2 );

    m_pclViewer->registerPointPickingCallback( PCLWidget::pickingEventHandler, m_pclViewer.get() );

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

// SlamWidget
SlamWidget::SlamWidget( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

SlamWidget::~SlamWidget()
{
    m_slamThread->terminate();
}

void SlamWidget::initialize()
{
    m_imagesWidget = new ImagesWidget( this );
    addWidget( m_imagesWidget );

    m_pclWidget = new PCLWidget( this );
    addWidget( m_pclWidget );

    m_slamThread = new SlamThread( this );

    connect( m_slamThread, &SlamThread::pointsImageSignal, m_imagesWidget, &ImagesWidget::setPointsImage );
    connect( m_slamThread, &SlamThread::tracksImageSignal, m_imagesWidget, &ImagesWidget::setTracksImage );

    connect( m_slamThread, &SlamThread::geometrySignal, this, &SlamWidget::setGeometry );

    connect( m_slamThread, &SlamThread::updateSignal, this, &SlamWidget::updateViews );

    m_slamThread->start();

}

void SlamWidget::setGeometry( const SlamGeometry &geo )
{
    m_pclWidget->setPointCloud( geo.points() );
}

void SlamWidget::updateViews()
{

}
