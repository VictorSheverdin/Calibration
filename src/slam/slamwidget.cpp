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
View3DWidget::View3DWidget( QWidget* parent )
    : PCLWidget( parent )
{
    initialize();
}

void View3DWidget::initialize()
{
    m_pclViewer->addCoordinateSystem( 0.5 );

    m_pclViewer->registerPointPickingCallback( View3DWidget::pickingEventHandler, m_pclViewer.get() );
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

void View3DWidget::setLeftPath( std::list< cv::Vec3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !m_leftTrajectoryActor ) {
        m_leftTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( m_leftTrajectoryActor );

    }

     m_leftTrajectoryActor->SetMapper( mapper );

}

void View3DWidget::setRightPath( std::list< cv::Vec3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !m_rightTrajectoryActor ) {
        m_rightTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( m_rightTrajectoryActor );

    }

    m_rightTrajectoryActor->SetMapper( mapper );

}

void View3DWidget::setPath( const std::list< StereoCameraMatrix > &path )
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

void View3DWidget::showPath( const bool value )
{
    if ( m_leftTrajectoryActor )
        m_leftTrajectoryActor->SetVisibility( value );

    if ( m_rightTrajectoryActor )
        m_rightTrajectoryActor->SetVisibility( value );

    if ( m_leftCameraActor )
        m_leftCameraActor->SetVisibility( value );

    if ( m_rightCameraActor )
        m_rightCameraActor->SetVisibility( value );

}

void View3DWidget::setFrustum( const StereoCameraMatrix &cameraMatrix )
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

void View3DWidget::setLeftFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !m_leftCameraActor ) {
        m_leftCameraActor = vtkSmartPointer< vtkActor >::New();
        m_leftCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( m_leftCameraActor );

    }

    m_leftCameraActor->SetMapper( mapper ) ;

}

void View3DWidget::setRightFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !m_rightCameraActor ) {
        m_rightCameraActor = vtkSmartPointer< vtkActor >::New();
        m_rightCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( m_rightCameraActor );

    }

    m_rightCameraActor->SetMapper( mapper ) ;

}

void View3DWidget::pickingEventHandler( const pcl::visualization::PointPickingEvent &event, void *viewer_void )
{
    float x, y, z;

    if ( event.getPointIndex() == -1 ) {
        return;
    }

    event.getPoint( x, y, z );

    auto distance = sqrt( x * x + y * y + z * z );

    std::stringstream ss;
    ss << distance;

    auto widget = reinterpret_cast< pcl::visualization::PCLVisualizer * >( viewer_void );

    widget->removeText3D( "Distance" );
    widget->addText3D( ss.str(), pcl::PointXYZ( x, y, z ), 0.1, 1.0, 0, 0, "Distance" );

}

// SlamViewWidget
SlamViewWidget::SlamViewWidget( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void SlamViewWidget::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );

    m_imagesWidget = new ImagesWidget( this );
    addWidget( m_imagesWidget );

    m_pclWidget = new View3DWidget( this );
    addWidget( m_pclWidget );

    int widthDiv2 = width() / 2;

    setSizes( QList< int >() << widthDiv2  << widthDiv2 );

}

void SlamViewWidget::setPath( const std::list< StereoCameraMatrix > &path )
{
    m_pclWidget->setPath( path );
}

void SlamViewWidget::setSparseCloud( const std::list< ColorPoint3d > &points )
{
    setPointCloud( points, "sparse_cloud" );
}

void SlamViewWidget::setPointCloud( const std::list< ColorPoint3d > &points, const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
{
    m_pclWidget->setPointCloud( points, id, origin, orientation );
}

void SlamViewWidget::setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
{
    m_pclWidget->setPointCloudPose( id, origin, orientation );
}

void SlamViewWidget::showPath( const bool flag )
{
    m_pclWidget->showPath( flag );
}

bool SlamViewWidget::contains( const std::string &id ) const
{
    return m_pclWidget->contains( id );
}

void SlamViewWidget::setPointsImage( const CvImage &image )
{
    m_imagesWidget->setPointsImage( image );
}

void SlamViewWidget::setTracksImage( const CvImage &image )
{
    m_imagesWidget->setTracksImage( image );
}

void SlamViewWidget::setStereoImage( const CvImage &image )
{
    m_imagesWidget->setStereoImage( image );
}

// SlamControlWidget
SlamControlWidget::SlamControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void SlamControlWidget::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Minimum );

    auto layout = new QHBoxLayout( this );

    m_viewOdometryCheck = new QCheckBox( tr( "Show odometry" ), this );
    m_viewOdometryCheck->setChecked( true );

    m_viewSparseCheck = new QCheckBox( tr( "Show sparse reconstruction" ), this );
    m_viewSparseCheck->setChecked( false );

    m_viewDenseCheck = new QCheckBox( tr( "Show dense reconstruction" ), this );
    m_viewDenseCheck->setChecked( true );

    layout->addWidget( m_viewOdometryCheck );
    layout->addWidget( m_viewSparseCheck );
    layout->addWidget( m_viewDenseCheck );

}

const QPointer< QCheckBox > &SlamControlWidget::odometryCheck() const
{
    return m_viewOdometryCheck;
}

const QPointer< QCheckBox > &SlamControlWidget::sparseCheck() const
{
    return m_viewSparseCheck;
}

const QPointer< QCheckBox > &SlamControlWidget::denseCheck() const
{
    return m_viewDenseCheck;
}

bool SlamControlWidget::isOdometryChecked() const
{
    return m_viewOdometryCheck->isChecked();
}

bool SlamControlWidget::isSparseChecked() const
{
    return m_viewSparseCheck->isChecked();
}

bool SlamControlWidget::isDenseChecked() const
{
    return m_viewDenseCheck->isChecked();
}

// SlamWidgetBase
SlamWidgetBase::SlamWidgetBase( const QString &calibrationFile, QWidget* parent )
    : QWidget( parent )
{
    initialize( calibrationFile );
}

SlamWidgetBase::~SlamWidgetBase()
{
    m_slamThread->terminate();
    m_slamThread->wait();
}

void SlamWidgetBase::initialize( const QString &calibrationFile )
{
    auto layout = new QVBoxLayout( this );

    m_controlWidget = new SlamControlWidget( this );
    m_viewWidget = new SlamViewWidget( this );

    layout->addWidget( m_controlWidget );
    layout->addWidget( m_viewWidget );

    StereoCalibrationDataShort calibration( calibrationFile.toStdString() );

    m_slamThread = new SlamThread( calibration, this );

    connect( m_slamThread, &SlamThread::updateSignal, this, &SlamWidgetBase::updateViews );

    m_slamThread->start();

    updateVisibility();

}

void SlamWidgetBase::updateVisibility()
{
    m_viewWidget->showPath( m_controlWidget->isOdometryChecked() );
}

std::list< StereoCameraMatrix > SlamWidgetBase::path() const
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

std::list< ColorPoint3d > SlamWidgetBase::sparseCloud() const
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

void SlamWidgetBase::updateViews()
{
    updateImages();
    update3dView();
}

void SlamWidgetBase::updateImages()
{
    m_viewWidget->setPointsImage( m_slamThread->pointsImage() );
    m_viewWidget->setTracksImage( m_slamThread->tracksImage() );
    m_viewWidget->setStereoImage( m_slamThread->stereoImage() );
}

void SlamWidgetBase::updatePath()
{
    m_viewWidget->setPath( path() );
}

void SlamWidgetBase::updateSparseCloud()
{
    m_viewWidget->setSparseCloud( sparseCloud() );
}

void SlamWidgetBase::updateDensePointCloud()
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

                if ( !m_viewWidget->contains( id ) )
                    m_viewWidget->setPointCloud( denseFrame->points(), id, origin, orientation );

                m_viewWidget->setPointCloudPose( id, origin, orientation );

            }

            ++counter;

/*
            auto stereoFrame = std::dynamic_pointer_cast< slam::StereoFrameBase >( *i );
            auto processedDenseFrame = std::dynamic_pointer_cast< slam::ProcessedDenseFrame >( *i );
            auto denseFrame = std::dynamic_pointer_cast< slam::DenseFrameBase >( *i );

            if ( denseFrame ) {

                std::string id;

                if ( processedDenseFrame )
                    id = "processed_cloud";
                else
                    id = "frame" + std::to_string( counter ) + "_cloud";

                auto projectionMatrix = stereoFrame->projectionMatrix();

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

                if ( processedDenseFrame || !m_pclWidget->contains( id ) )
                    setPointCloud( denseFrame->points(), id, origin, orientation );

                setPointCloudPose( id, origin, orientation );

            }

            ++counter;*/

        }

    }

}

void SlamWidgetBase::update3dView()
{
    updatePath();
    updateSparseCloud();
    updateDensePointCloud();
}

// SlamImageWidget
SlamImageWidget::SlamImageWidget( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, QWidget* parent )
    : SlamWidgetBase( calibrationFile, parent )
{
    initialize();

    setImageList( leftList, rightList );
}

void SlamImageWidget::initialize()
{
    m_index = 0;
    startTimer( 100 );    
}

void SlamImageWidget::setImageList( const QStringList &leftList, const QStringList &rightList )
{
    if ( leftList.size() == rightList.size() ) {
        m_leftList = leftList;
        m_rightList = rightList;

        m_index = 0;

    }

}

void SlamImageWidget::timerEvent( QTimerEvent * )
{
    if ( m_index < m_leftList.size() ) {

        CvImage leftImage( m_leftList[ m_index ].toStdString() );
        CvImage rightImage( m_rightList[ m_index ].toStdString() );

        m_slamThread->process( leftImage, rightImage );
        ++m_index;

    }

}

// SlamCameraWidget
SlamCameraWidget::SlamCameraWidget( const QString &leftCameraIp, const QString &rightCameraIp, const QString &calibrationFile, QWidget* parent )
    : SlamWidgetBase( calibrationFile, parent ), m_camera( leftCameraIp.toStdString(), rightCameraIp.toStdString(), parent )
{
    initialize();
}

void SlamCameraWidget::initialize()
{
    connect( &m_camera, &StereoCamera::receivedFrame, this, &SlamCameraWidget::updateFrame );
}

void SlamCameraWidget::updateFrame()
{
    auto frame = m_camera.getFrame();

    if ( !frame.empty() )
        m_slamThread->process( frame.leftFrame(), frame.rightFrame() );

}

