#include "src/common/precompiled.h"

#include "slamwidget.h"

#include "slamthread.h"
#include "map.h"
#include "frame.h"

#include <vtkPointPicker.h>

#include <vtkGenericOpenGLRenderWindow.h>

#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPlanes.h>
#include <vtkFrustumSource.h>

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

// PCLWidget
View3dWidget::View3dWidget( QWidget* parent )
    : PCLWidget( parent )
{
    initialize();
}

void View3dWidget::initialize()
{
}

vtkSmartPointer< vtkPolyDataMapper > polyLineMapper( std::list< cv::Vec3d > &points )
{
    vtkNew< vtkPoints > pts;

    for ( auto &i : points )
        pts->InsertNextPoint( i[ 0 ], i[ 1 ], i[ 2 ] );

    vtkNew< vtkPolyLine > polyLine;
     polyLine->GetPointIds()->SetNumberOfIds( points.size() );
     for( unsigned int i = 0; i < points.size(); ++i )
        polyLine->GetPointIds()->SetId( i, i );

     vtkNew< vtkCellArray > cells;
     cells->InsertNextCell( polyLine );

     vtkNew< vtkPolyData > polyData;
     polyData->SetPoints( pts );
     polyData->SetLines( cells );

     vtkNew< vtkPolyDataMapper > mapper;
     mapper->SetInputData( polyData );

     return mapper;
}

void View3dWidget::setLeftPath( std::list< cv::Vec3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !m_leftTrajectoryActor ) {
        m_leftTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( m_leftTrajectoryActor );

    }

     m_leftTrajectoryActor->SetMapper( mapper );

}

void View3dWidget::setRightPath( std::list< cv::Vec3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !m_rightTrajectoryActor ) {
        m_rightTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( m_rightTrajectoryActor );

    }

    m_rightTrajectoryActor->SetMapper( mapper );

}

void View3dWidget::setPath( const std::list< StereoCameraMatrix > &path )
{
    std::list< cv::Vec3d > leftPoints;
    std::list< cv::Vec3d > rightPoints;

    for ( auto &i : path ) {

        auto leftProjectionMatrix = i.leftProjectionMatrix();
        auto rightProjectionMatrix = i.rightProjectionMatrix();

        cv::Mat leftTranslation = -leftProjectionMatrix.rotation().t() * leftProjectionMatrix.translation();
        cv::Mat rightTranslation = -rightProjectionMatrix.rotation().t() * rightProjectionMatrix.translation();

        leftPoints.push_back( cv::Vec3d( leftTranslation.at< double >( 0, 0 ), leftTranslation.at< double >( 1, 0 ), leftTranslation.at< double >( 2, 0 ) ) );
        rightPoints.push_back( cv::Vec3d( rightTranslation.at< double >( 0, 0 ), rightTranslation.at< double >( 1, 0 ), rightTranslation.at< double >( 2, 0 ) ) );

    }

    setLeftPath( leftPoints );
    setRightPath( rightPoints );

    if ( !path.empty() )
        setFrustum( path.back() );

    update();

}

void View3dWidget::setFrustum( const StereoCameraMatrix &cameraMatrix )
{
    setLeftFrustum( cameraMatrix.leftProjectionMatrix() );
    setRightFrustum( cameraMatrix.rightProjectionMatrix() );
}

vtkSmartPointer< vtkPolyDataMapper > cameraMapper( const ProjectionMatrix &cameraMatrix )
{
    double fx = cameraMatrix.fx();
    double fy = cameraMatrix.fy();
    double cy = cameraMatrix.cy();

    double fovy = 2.0 * atan2( cy, fy ) * 180.0 / CV_PI;
    double aspectRatio = fy / fx;
    double scale = 0.2;

    vtkNew< vtkCamera > camera;

    auto rot = cameraMatrix.rotation().t();

    cv::Mat translation = -rot * cameraMatrix.translation();
    cv::Vec3d translationVec( translation.at< double >( 0, 0 ), translation.at< double >( 1, 0 ), translation.at< double >( 2, 0 ) );

    cv::Mat upVec = ( cv::Mat_< double >( 3, 1 ) << 0.0, 1.0, 0.0 );
    cv::Mat focalVec = ( cv::Mat_< double >( 3, 1 ) << 0.0, 0.0, 1.0 );

    cv::Mat rotUpVec = rot * upVec;
    cv::Mat rotFocalVec = rot * focalVec;

    camera->SetViewAngle( fovy );
    camera->SetPosition( translationVec[ 0 ], translationVec[ 1 ], translationVec[ 2 ] );
    camera->SetViewUp( rotUpVec.at< double >( 0 ), rotUpVec.at< double >( 1 ), rotUpVec.at< double >( 2 ) );
    camera->SetFocalPoint( translationVec[ 0 ] + rotFocalVec.at< double >( 0 ),
                                    translationVec[ 1 ]+ rotFocalVec.at< double >( 1 ),
                                    translationVec[ 2 ] + rotFocalVec.at< double >( 2 ) );
    camera->SetClippingRange( 1e-9, scale );

    double planesArray[ 24 ];
    camera->GetFrustumPlanes( aspectRatio, planesArray );

    vtkNew< vtkPlanes > planes;
    planes->SetFrustumPlanes( planesArray );

    vtkNew< vtkFrustumSource > frustumSource;
    frustumSource->ShowLinesOff();
    frustumSource->SetPlanes( planes );
    frustumSource->Update();

    vtkPolyData* frustum = frustumSource->GetOutput();

    vtkNew< vtkPolyDataMapper > mapper;
    mapper->SetInputData( frustum );

    return mapper;
}

void View3dWidget::setLeftFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !m_leftCameraActor ) {
        m_leftCameraActor = vtkSmartPointer< vtkActor >::New();
        m_leftCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( m_leftCameraActor );

    }

    m_leftCameraActor->SetMapper( mapper ) ;

}

void View3dWidget::setRightFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !m_rightCameraActor ) {
        m_rightCameraActor = vtkSmartPointer< vtkActor >::New();
        m_rightCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( m_rightCameraActor );

    }

    m_rightCameraActor->SetMapper( mapper ) ;

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
    m_slamThread->wait();
}

void SlamWidget::initialize()
{
    m_imagesWidget = new ImagesWidget( this );
    addWidget( m_imagesWidget );

    m_pclWidget = new View3dWidget( this );
    addWidget( m_pclWidget );

    int widthDiv2 = width() / 2;

    setSizes( QList< int >() << widthDiv2  << widthDiv2 );

    m_slamThread = new SlamThread( this );

    connect( m_slamThread, &SlamThread::updateSignal, this, &SlamWidget::updateViews );

    m_slamThread->start();

    startTimer( 1000 );
}

void SlamWidget::setPath( const std::list< StereoCameraMatrix > &path )
{
    m_pclWidget->setPath( path );
}

void SlamWidget::setSparseCloud( const std::list< ColorPoint3d > &points )
{
    setPointCloud( points, "sparse_cloud" );
}

void SlamWidget::setPointCloud( const std::list< ColorPoint3d > &points, const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
{
    m_pclWidget->setPointCloud( points, id, origin, orientation );
}

void SlamWidget::setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
{
    m_pclWidget->setPointCloudPose( id, origin, orientation );
}

void SlamWidget::timerEvent( QTimerEvent * )
{
}

std::list< StereoCameraMatrix > SlamWidget::path() const
{
    std::list< StereoCameraMatrix > ret;

    auto maps = m_slamThread->maps();

    for ( auto &map : maps ) {

        auto frames = map->frames();

        for ( auto &i : frames ) {

            auto stereoFrame = std::dynamic_pointer_cast< slam::StereoFrame >( i );

            if ( stereoFrame )
                ret.push_back( stereoFrame->projectionMatrix() );

        }

    }

    return ret;
}

std::list< ColorPoint3d > SlamWidget::sparseCloud() const
{
    std::list< ColorPoint3d > ret;

    auto maps = m_slamThread->maps();

    for ( auto &map : maps ) {

        auto mapPoints = map->mapPoints();

        for ( auto &i : mapPoints ) {

            if ( i )
                ret.push_back( ColorPoint3d( i->point(), i->color() ) );

        }

    }

    return ret;
}

void SlamWidget::updateViews()
{
    updateImages();
    update3dView();
}

void SlamWidget::updateImages()
{
    m_imagesWidget->setPointsImage( m_slamThread->pointsImage() );
    m_imagesWidget->setTracksImage( m_slamThread->tracksImage() );
    m_imagesWidget->setStereoImage( m_slamThread->stereoImage() );
}

void SlamWidget::updatePath()
{
    setPath( path() );
}

void SlamWidget::updateSparseCloud()
{
    setSparseCloud( sparseCloud() );
}

void SlamWidget::updateDensePointCloud()
{
    auto maps = m_slamThread->maps();

    int counter = 0;

    for ( auto &map : maps ) {

        auto frames = map->frames();

        for ( auto i = frames.begin(); i != frames.end(); ++i ) {

            auto denseFrame = std::dynamic_pointer_cast< slam::DenseFrame >( *i );

            if ( denseFrame ) {

                auto id = "frame" + std::to_string( counter ) + "_cloud";

                auto projectionMatrix = denseFrame->projectionMatrix();

                cv::Mat rotation = projectionMatrix.leftProjectionMatrix().rotation().t();
                cv::Mat translation = -rotation * projectionMatrix.leftProjectionMatrix().translation();

                Eigen::Vector4f origin( translation.at< double >( 0, 0 ), translation.at< double >( 0, 1 ), translation.at< double >( 0, 2 ), 1 );

                Eigen::Quaternionf::RotationMatrixType rotMatrix;

                rotMatrix( 0, 0 ) = rotation.at< double >( 0, 0 );
                rotMatrix( 0, 1 ) = rotation.at< double >( 0, 1 );
                rotMatrix( 0, 2 ) = rotation.at< double >( 0, 2 );
                rotMatrix( 1, 0 ) = rotation.at< double >( 1, 0 );
                rotMatrix( 1, 1 ) = rotation.at< double >( 1, 1 );
                rotMatrix( 1, 2 ) = rotation.at< double >( 1, 2 );
                rotMatrix( 2, 0 ) = rotation.at< double >( 2, 0 );
                rotMatrix( 2, 1 ) = rotation.at< double >( 2, 1 );
                rotMatrix( 2, 2 ) = rotation.at< double >( 2, 2 );

                Eigen::Quaternionf orientation( rotMatrix );

                if ( !m_pclWidget->contains( id ) )
                    setPointCloud( denseFrame->points(), id, origin, orientation );

                setPointCloudPose( id, origin, orientation );

            }

            ++counter;

        }

    }

}

void SlamWidget::update3dView()
{
    updatePath();
    // updateSparseCloud();
    updateDensePointCloud();
}
