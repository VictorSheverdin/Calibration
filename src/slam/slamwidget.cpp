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

    m_stereoWidget = new ImageWidget( this );
    addWidget( m_stereoWidget );

    m_stereoWidget->hide();

    int widthDiv3 = width() / 3;

    setSizes( QList< int >() << widthDiv3 << widthDiv3 << widthDiv3 );

}

void ImagesWidget::setPointsImage( const CvImage &image )
{
    m_pointsWidget->setImage( image );
}

void ImagesWidget::setTracksImage( const CvImage &image )
{
    m_tracksWidget->setImage( image );
}

void ImagesWidget::setStereoImage( const CvImage &image )
{
    m_stereoWidget->setImage( image );
}

// PCLViewer
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

void PCLWidget::setLeftPath( const pcl::PointCloud< pcl::PointXYZ >::Ptr path )
{
    std::string id = "leftPath";

    if( !m_pclViewer->updatePointCloud( path, id ) ) {
        m_pclViewer->addPointCloud( path, id );
        m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id );
    }

    update();

}

void PCLWidget::setRightPath( const pcl::PointCloud< pcl::PointXYZ >::Ptr path )
{
    std::string id = "rightPath";

    if( !m_pclViewer->updatePointCloud( path, id ) ) {
        m_pclViewer->addPointCloud( path, id );
        m_pclViewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id );
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

    int widthDiv2 = width() / 2;

    setSizes( QList< int >() << widthDiv2  << widthDiv2 );

    m_slamThread = new SlamThread( this );

    connect( m_slamThread, &SlamThread::updateSignal, this, &SlamWidget::updateViews );

    m_slamThread->start();

}

void SlamWidget::setGeometry( const SlamGeometry &geo )
{
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );

    for ( auto &i : geo.points() ) {

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

    m_pclWidget->setPointCloud( pointCloud );

    pcl::PointCloud<pcl::PointXYZ>::Ptr leftTrajectory( new pcl::PointCloud< pcl::PointXYZ > );
    pcl::PointCloud<pcl::PointXYZ>::Ptr rightTrajectory( new pcl::PointCloud< pcl::PointXYZ > );

    for ( auto &i : geo.path() ) {
        auto leftProjectionMatrix = i.leftProjectionMatrix();
        auto rightProjectionMatrix = i.rightProjectionMatrix();

        cv::Mat leftTranslation = -leftProjectionMatrix.rotation().t() * leftProjectionMatrix.translation();
        cv::Mat rightTranslation = -rightProjectionMatrix.rotation().t() * rightProjectionMatrix.translation();

        leftTrajectory->push_back( pcl::PointXYZ( leftTranslation.at< double >( 0, 0 ), leftTranslation.at< double >( 1, 0 ), leftTranslation.at< double >( 2, 0 ) ) );
        rightTrajectory->push_back( pcl::PointXYZ( rightTranslation.at< double >( 0, 0 ), rightTranslation.at< double >( 1, 0 ), rightTranslation.at< double >( 2, 0 ) ) );
    }

    m_pclWidget->setLeftPath( leftTrajectory );
    // m_pclWidget->setRightPath( rightTrajectory );

}

void SlamWidget::updateViews()
{
    m_imagesWidget->setPointsImage( m_slamThread->pointsImage() );
    m_imagesWidget->setTracksImage( m_slamThread->tracksImage() );
    m_imagesWidget->setStereoImage( m_slamThread->stereoImage() );

    setGeometry( m_slamThread->geometry() );
}
