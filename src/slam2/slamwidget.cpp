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

    _pointsWidget->hide();
    _stereoWidget->hide();

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

void ReconstructionViewWidget::setLeftPath( std::list< cv::Vec3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !_leftTrajectoryActor ) {
        _leftTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( _leftTrajectoryActor );

    }

     _leftTrajectoryActor->SetMapper( mapper );

}

void ReconstructionViewWidget::setRightPath( std::list< cv::Vec3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !_rightTrajectoryActor ) {
        _rightTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( _rightTrajectoryActor );

    }

    _rightTrajectoryActor->SetMapper( mapper );

}

void ReconstructionViewWidget::showPath( const bool value )
{
    if ( _leftTrajectoryActor )
        _leftTrajectoryActor->SetVisibility( value );

    if ( _rightTrajectoryActor )
        _rightTrajectoryActor->SetVisibility( value );

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

void SlamViewWidget::setSparseCloud( const std::list< ColorPoint3d > &points )
{
    setPointCloud( points, "sparse_cloud" );
}

void SlamViewWidget::setPointCloud( const std::list< ColorPoint3d > &points, const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation )
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

    _processorThread = new ProcessorThread( this );

    _updateTimer = new QTimer( this );
    _updateTimer->start( 1000 / 20 );

    connect( _updateTimer, &QTimer::timeout, this, &SlamWidgetBase::updateViews );

    _processorThread->start();

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
    _fps = 20;

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

        time += std::chrono::milliseconds{ static_cast< int64_t >( 1000.0 / _fps  ) } ;

    }

}
