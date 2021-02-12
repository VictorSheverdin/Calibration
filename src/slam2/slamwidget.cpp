#include "src/common/precompiled.h"

#include "slamwidget.h"

#include "map.h"
#include "frame.h"

#include "thread.h"

#include "src/common/calibrationdatabase.h"

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

// #define _DEBUG_THREADS

// ImagesWidget
ImagesWidget::ImagesWidget( QWidget* parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void ImagesWidget::initialize()
{
    _pointsWidget = new ImageWidget( this );
    addWidget( _pointsWidget );

    _tracksWidget = new ImageWidget( this );
    addWidget( _tracksWidget );

    _stereoWidget = new ImageWidget( this );
    addWidget( _stereoWidget );

    //_pointsWidget->hide();
    //_stereoWidget->hide();

    int widthDiv3 = width() / 3;

    setSizes( QList< int >() << widthDiv3 << widthDiv3 << widthDiv3 );

}

void ImagesWidget::setPointsImage( const CvImage &image )
{
    _pointsWidget->setImage( image );
}

void ImagesWidget::setTracksImage( const CvImage &image )
{
    _tracksWidget->setImage( image );
}

void ImagesWidget::setStereoImage( const CvImage &image )
{
    _stereoWidget->setImage( image );
}

// PCLWidget
ReconstructionViewWidget::ReconstructionViewWidget( QWidget* parent )
    : PCLWidget( parent )
{
    initialize();
}

void ReconstructionViewWidget::initialize()
{
    m_pclViewer->setCameraPosition( 0, 0, -10, 0, -1, 0 );
    m_pclViewer->setCameraClipDistances( 0.1, 10000 );

    m_pclViewer->addCoordinateSystem( 0.5 );

    m_pclViewer->registerPointPickingCallback( ReconstructionViewWidget::pickingEventHandler, m_pclViewer.get() );
}

vtkSmartPointer< vtkPolyDataMapper > polyLineMapper( const std::vector< cv::Point3d > &points )
{
    vtkNew< vtkPoints > pts;

    for ( auto &i : points )
        pts->InsertNextPoint( i.x, i.y, i.z );

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

cv::Point3d calcTranslation( const ProjectionMatrix &cameraMatrix )
{
    auto rot = cameraMatrix.rotation().t();

    cv::Mat translation = -rot * cameraMatrix.translation();
    cv::Point3d translationVec( translation.at< double >( 0, 0 ), translation.at< double >( 1, 0 ), translation.at< double >( 2, 0 ) );

    return translationVec;
}

void ReconstructionViewWidget::setPath( const std::vector< StereoProjectionMatrix > &points )
{
    std::vector< cv::Point3d > leftPoints, rightPoints;

    for ( auto &i : points ) {
        leftPoints.push_back( calcTranslation( i.leftProjectionMatrix() ) );
        rightPoints.push_back( calcTranslation( i.rightProjectionMatrix() ) );
    }

    setLeftPath( leftPoints );
    setRightPath( rightPoints );

}

void ReconstructionViewWidget::setLeftPath( const std::vector< cv::Point3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !_leftTrajectoryActor ) {
        _leftTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( _leftTrajectoryActor );

    }

     _leftTrajectoryActor->SetMapper( mapper );

}

void ReconstructionViewWidget::setRightPath( const std::vector< cv::Point3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !_rightTrajectoryActor ) {
        _rightTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( _rightTrajectoryActor );

    }

     _rightTrajectoryActor->SetMapper( mapper );

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

    auto translationVec = calcTranslation( cameraMatrix );

    cv::Mat upVec = ( cv::Mat_< double >( 3, 1 ) << 0.0, 1.0, 0.0 );
    cv::Mat focalVec = ( cv::Mat_< double >( 3, 1 ) << 0.0, 0.0, 1.0 );

    cv::Mat rotUpVec = rot * upVec;
    cv::Mat rotFocalVec = rot * focalVec;

    camera->SetViewAngle( fovy );
    camera->SetPosition( translationVec.x, translationVec.y, translationVec.z );
    camera->SetViewUp( rotUpVec.at< double >( 0 ), rotUpVec.at< double >( 1 ), rotUpVec.at< double >( 2 ) );
    camera->SetFocalPoint( translationVec.x + rotFocalVec.at< double >( 0 ),
                                    translationVec.y + rotFocalVec.at< double >( 1 ),
                                    translationVec.z + rotFocalVec.at< double >( 2 ) );
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

void ReconstructionViewWidget::setFrustum( const StereoProjectionMatrix &cameraMatrix )
{
    setLeftFrustum( cameraMatrix.leftProjectionMatrix() );
    setRightFrustum( cameraMatrix.rightProjectionMatrix() );
}

void ReconstructionViewWidget::setLeftFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !_leftCameraActor ) {
        _leftCameraActor = vtkSmartPointer< vtkActor >::New();
        _leftCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( _leftCameraActor );

    }

    _leftCameraActor->SetMapper( mapper ) ;

}

void ReconstructionViewWidget::setRightFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !_rightCameraActor ) {
        _rightCameraActor = vtkSmartPointer< vtkActor >::New();
        _rightCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( _rightCameraActor );

    }

    _rightCameraActor->SetMapper( mapper ) ;

}

void ReconstructionViewWidget::showPath( const bool value )
{
    if ( _leftTrajectoryActor )
        _leftTrajectoryActor->SetVisibility( value );

    if ( _rightTrajectoryActor )
        _rightTrajectoryActor->SetVisibility( value );
}

void ReconstructionViewWidget::showFrustum( const bool value )
{
    if ( _leftCameraActor )
        _leftCameraActor->SetVisibility( value );

    if ( _rightCameraActor )
        _rightCameraActor->SetVisibility( value );
}

void ReconstructionViewWidget::pickingEventHandler( const pcl::visualization::PointPickingEvent &event, void *viewer_void )
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

    _imagesWidget = new ImagesWidget( this );
    addWidget( _imagesWidget );

    _view3dWidget = new ReconstructionViewWidget( this );
    addWidget( _view3dWidget );

    int widthDiv2 = width() / 2;

    setSizes( QList< int >() << widthDiv2  << widthDiv2 );

}

void SlamViewWidget::setPath( const std::vector< StereoProjectionMatrix > &path )
{
    _view3dWidget->setPath( path );
}

void SlamViewWidget::setCamera( const StereoProjectionMatrix &camera )
{
    _view3dWidget->setFrustum( camera );
}

void SlamViewWidget::setSparseCloud( const std::vector< ColorPoint3d > &points )
{
    setPointCloud( points, "sparse_cloud" );
}

void SlamViewWidget::setPointCloud( const std::vector< ColorPoint3d > &points, const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
{
    _view3dWidget->setPointCloud( points, id, origin, orientation );
}

void SlamViewWidget::setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
{
    _view3dWidget->setPointCloudPose( id, origin, orientation );
}

void SlamViewWidget::showPath( const bool flag )
{
    _view3dWidget->showPath( flag );
}

bool SlamViewWidget::contains( const std::string &id ) const
{
    return _view3dWidget->contains( id );
}

void SlamViewWidget::setPointsImage( const CvImage &image )
{
    _imagesWidget->setPointsImage( image );
}

void SlamViewWidget::setTracksImage( const CvImage &image )
{
    _imagesWidget->setTracksImage( image );
}

void SlamViewWidget::setStereoImage( const CvImage &image )
{
    _imagesWidget->setStereoImage( image );
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

    _viewOdometryCheck = new QCheckBox( tr( "Show odometry" ), this );
    _viewOdometryCheck->setChecked( true );

    _viewSparseCheck = new QCheckBox( tr( "Show sparse reconstruction" ), this );
    _viewSparseCheck->setChecked( false );

    _viewDenseCheck = new QCheckBox( tr( "Show dense reconstruction" ), this );
    _viewDenseCheck->setChecked( true );

    layout->addWidget( _viewOdometryCheck );
    layout->addWidget( _viewSparseCheck );
    layout->addWidget( _viewDenseCheck );

}

const QPointer< QCheckBox > &SlamControlWidget::odometryCheck() const
{
    return _viewOdometryCheck;
}

const QPointer< QCheckBox > &SlamControlWidget::sparseCheck() const
{
    return _viewSparseCheck;
}

const QPointer< QCheckBox > &SlamControlWidget::denseCheck() const
{
    return _viewDenseCheck;
}

bool SlamControlWidget::isOdometryChecked() const
{
    return _viewOdometryCheck->isChecked();
}

bool SlamControlWidget::isSparseChecked() const
{
    return _viewSparseCheck->isChecked();
}

bool SlamControlWidget::isDenseChecked() const
{
    return _viewDenseCheck->isChecked();
}

// SlamWidgetBase
SlamWidgetBase::SlamWidgetBase( const QString &calibrationFile, QWidget* parent )
    : QWidget( parent )
{
    initialize( calibrationFile );
}

SlamWidgetBase::~SlamWidgetBase()
{
    _processorThread->requestInterruption();
    _processorThread->wait();
}

void SlamWidgetBase::initialize( const QString &calibrationFile )
{
    auto layout = new QVBoxLayout( this );

    _controlWidget = new SlamControlWidget( this );
    _viewWidget = new SlamViewWidget( this );

    layout->addWidget( _controlWidget );
    layout->addWidget( _viewWidget );

    StereoCalibrationDataShort calibration( calibrationFile.toStdString() );

    slam2::Parameters parameters;

    parameters.setCalibration( calibration );

    _processorThread = new ProcessorThread( parameters, this );

    _updateTimer = new QTimer( this );
    _updateTimer->start( 1000 / 20 );

    connect( _updateTimer, &QTimer::timeout, this, &SlamWidgetBase::updateViews );

#ifndef _DEBUG_THREADS
    _processorThread->start();
#endif

    updateVisibility();

}

void SlamWidgetBase::updateVisibility()
{
    _viewWidget->showPath( _controlWidget->isOdometryChecked() );
}

void SlamWidgetBase::updateViews()
{
    updateImages();
    update3DView();
}

void SlamWidgetBase::updateImages()
{
    _viewWidget->setPointsImage( _processorThread->pointsImage() );
    _viewWidget->setTracksImage( _processorThread->tracksImage() );
    _viewWidget->setStereoImage( _processorThread->stereoImage() );
}

void SlamWidgetBase::update3DView()
{
    _viewWidget->setSparseCloud( _processorThread->sparseCloud() );

    auto path = _processorThread->path();

    _viewWidget->setPath( _processorThread->path() );

    if ( !path.empty() )
        _viewWidget->setCamera( path.back() );

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
    _index = 0;
    _fps = 25;

    startTimer( 1000. / _fps );
}

void SlamImageWidget::setImageList( const QStringList &leftList, const QStringList &rightList )
{
    if ( leftList.size() == rightList.size() ) {
        _leftList = leftList;
        _rightList = rightList;

        _index = 0;

    }

}

double SlamImageWidget::fps() const
{
    return _fps;
}

void SlamImageWidget::fps( const double value )
{
    _fps = value;
}

void SlamImageWidget::timerEvent( QTimerEvent * )
{
    static std::chrono::time_point< std::chrono::system_clock > time = std::chrono::system_clock::now();

    if ( _index < _leftList.size() ) {

        std::cout << _leftList[ _index ].toStdString() << " " << _rightList[ _index ].toStdString() << std::endl;

        StampedImage leftImage( time, _leftList[ _index ].toStdString() );
        StampedImage rightImage( time, _rightList[ _index ].toStdString() );

        _processorThread->process( StampedStereoImage( leftImage, rightImage ) );
        ++_index;

#ifdef _DEBUG_THREADS
        _processorThread->processNext();
#endif

        time += std::chrono::milliseconds{ static_cast< int64_t >( 1000.0 / _fps  ) } ;

    }

}
