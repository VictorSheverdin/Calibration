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

    m_pointsWidget->hide();
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

    if ( !m_leftTrajectoryActor ) {
        m_leftTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( m_leftTrajectoryActor );

    }

     m_leftTrajectoryActor->SetMapper( mapper );

}

void ReconstructionViewWidget::setRightPath( std::list< cv::Vec3d > &points )
{
    auto mapper = polyLineMapper( points );

    if ( !m_rightTrajectoryActor ) {
        m_rightTrajectoryActor = vtkSmartPointer< vtkActor >::New();
        m_renderer->AddActor( m_rightTrajectoryActor );

    }

    m_rightTrajectoryActor->SetMapper( mapper );

}

void ReconstructionViewWidget::setPath( const std::list< StereoProjectionMatrix > &path )
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

void ReconstructionViewWidget::showPath( const bool value )
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

void ReconstructionViewWidget::setFrustum( const StereoProjectionMatrix &cameraMatrix )
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

void ReconstructionViewWidget::setLeftFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !m_leftCameraActor ) {
        m_leftCameraActor = vtkSmartPointer< vtkActor >::New();
        m_leftCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( m_leftCameraActor );

    }

    m_leftCameraActor->SetMapper( mapper ) ;

}

void ReconstructionViewWidget::setRightFrustum( const ProjectionMatrix &cameraMatrix )
{
    auto mapper = cameraMapper( cameraMatrix );

    if ( !m_rightCameraActor ) {
        m_rightCameraActor = vtkSmartPointer< vtkActor >::New();
        m_rightCameraActor->GetProperty()->SetRepresentationToWireframe();

        m_renderer->AddActor( m_rightCameraActor );

    }

    m_rightCameraActor->SetMapper( mapper ) ;

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

// ImuView
ImuView::ImuView( QWidget* parent )
    : PCLWidget( parent )
{
    initialize();
}

void ImuView::initialize()
{
    m_pclViewer->setCameraPosition( 10, 10, 0, 0, 0, 1 );
    m_pclViewer->setCameraClipDistances( 0.1, 10000 );

    m_pclViewer->addCoordinateSystem( 2.0, "world" );
    m_pclViewer->addCoordinateSystem( 1.0, "imu" );
    m_pclViewer->addCube( -.2, .2, -.8, .8, -.1, .1, 0.5, 0.5, 0.5, "beam" );
}

Eigen::Quaterniond ImuView::alignQuaternion( const Eigen::Vector3d &value )
{
    auto norm = value.normalized();

    if ( ! norm.isZero() ) {

        auto xCos = sqrt( 1. - norm.y() * norm.y() );
        auto yCos = sqrt( 1. - norm.x() * norm.x() );

        return Eigen::Quaterniond( sqrt( 0.5 * ( 1. + xCos ) ), sqrt( 0.5 * ( 1. - xCos ) ), 0, 0 ) * Eigen::Quaterniond( sqrt( 0.5 * ( 1. + yCos ) ), 0, -sqrt( 0.5 * ( 1. - yCos ) ), 0 );

    }

    return Eigen::Quaterniond();

}

void ImuView::setPose( const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation )
{
    Eigen::Affine3f pose( rotation.cast< float >() );

    pose.translation() = translation.cast< float >();

    m_pclViewer->updateShapePose( "beam", pose );
    m_pclViewer->updateCoordinateSystemPose( "imu", pose );

    update();

}

void ImuView::setAccelerationVector( const Eigen::Vector3d &value )
{
    if ( m_pclViewer->contains( "gravity") )
        m_pclViewer->removeShape( "gravity" );

    m_pclViewer->addArrow( pcl::PointXYZ( value.x(), value.y(), value.z() ), pcl::PointXYZ( 0., 0., 0. ), 0., 1., 0., true, "gravity" );

}

void ImuView::setMagVector( const Eigen::Vector3d &value )
{
    if ( m_pclViewer->contains( "magnitometer") )
        m_pclViewer->removeShape( "magnitometer" );

    m_pclViewer->addArrow( pcl::PointXYZ( value.x(), value.y(), value.z() ), pcl::PointXYZ( 0., 0., 0. ), 0., 0., 1., true, "magnitometer" );

}

// ImuChart
ImuChart::ImuChart( const QString &title, QWidget *parent )
    : QWidget( parent )
{
    initialize( title );
}

void ImuChart::initialize( const QString &title )
{
    setSizePolicy( QSizePolicy::Maximum, QSizePolicy::Maximum );

    _minT = DOUBLE_MAX;
    _maxT = -DOUBLE_MAX;

    _chartView = new QtCharts::QChartView( this );

    _chartView->setRenderHint( QPainter::Antialiasing );

    _chart = new QtCharts::QChart();

    auto checkLayout = new QHBoxLayout();

    _xCheck = new QCheckBox( tr( "x" ), this );
    _xCheck->setChecked( true );

    _yCheck = new QCheckBox( tr( "y" ), this );
    _yCheck->setChecked( true );

    _zCheck = new QCheckBox( tr( "z" ), this );
    _zCheck->setChecked( true );

    checkLayout->addWidget( _xCheck );
    checkLayout->addWidget( _yCheck );
    checkLayout->addWidget( _zCheck );

    QPen xPen;
    xPen.setWidth( 1 );
    xPen.setColor( Qt::red );

    _xSeries = new QLineSeries();
    _xSeries->setPen( xPen );

    QPen yPen;
    yPen.setWidth( 1 );
    yPen.setColor( Qt::green );

    _ySeries = new QLineSeries();
    _ySeries->setPen( yPen );

    QPen zPen;
    zPen.setWidth( 1 );
    zPen.setColor( Qt::blue );

    _zSeries = new QLineSeries();
    _zSeries->setPen( zPen );

    _xSeries->setUseOpenGL( true );
    _ySeries->setUseOpenGL( true );
    _zSeries->setUseOpenGL( true );

    _xSeries->setName( "x" );
    _ySeries->setName( "y" );
    _zSeries->setName( "z" );

    _chart->createDefaultAxes();
    _chart->setTitle( title );
    _chart->addSeries( _xSeries );
    _chart->addSeries( _ySeries );
    _chart->addSeries( _zSeries );

    _xAxis = new QValueAxis();
    _yAxis = new QValueAxis();

    _chart->addAxis( _xAxis, Qt::AlignBottom );
    _chart->addAxis( _yAxis, Qt::AlignLeft );

    _xSeries->attachAxis( _xAxis );
    _ySeries->attachAxis( _xAxis );
    _zSeries->attachAxis( _xAxis );

    _xSeries->attachAxis( _yAxis );
    _ySeries->attachAxis( _yAxis );
    _zSeries->attachAxis( _yAxis );

    _chartView->setChart( _chart );

    QVBoxLayout *layout = new QVBoxLayout( this );
    layout->addLayout( checkLayout );
    layout->addWidget( _chartView );

    connect( _xCheck, &QCheckBox::stateChanged , this, &ImuChart::updateVisibility );
    connect( _yCheck, &QCheckBox::stateChanged , this, &ImuChart::updateVisibility );
    connect( _zCheck, &QCheckBox::stateChanged , this, &ImuChart::updateVisibility );

    updateVisibility();
}

void ImuChart::addValue( const double t, const Eigen::Vector3d &value )
{
    _xSeries->append( t, value.x() );
    _ySeries->append( t, value.y() );
    _zSeries->append( t, value.z() );

    auto xCount = _xSeries->count();
    if ( xCount > _maxCount )
        _xSeries->removePoints( 0, xCount - _maxCount );

    auto yCount = _ySeries->count();
    if ( yCount > _maxCount )
        _ySeries->removePoints( 0, yCount - _maxCount );

    auto zCount = _zSeries->count();
    if ( zCount > _maxCount )
        _zSeries->removePoints( 0, zCount - _maxCount );

    updateMinMax();
}

void ImuChart::clear()
{
    _xSeries->clear();
    _ySeries->clear();
    _zSeries->clear();

    updateMinMax();
}

QList<QPointF> ImuChart::xPoints() const
{
    return _xSeries->points();
}

QList<QPointF> ImuChart::yPoints() const
{
    return _ySeries->points();
}

QList<QPointF> ImuChart::zPoints() const
{
    return _zSeries->points();
}

void ImuChart::updateVisibility()
{
    _xSeries->setVisible( _xCheck->isChecked() );
    _ySeries->setVisible( _yCheck->isChecked() );
    _zSeries->setVisible( _zCheck->isChecked() );

    updateMinMax();

}

void ImuChart::updateMinMax()
{
    _minT = _minValue = DOUBLE_MAX;
    _maxT = _maxValue = -DOUBLE_MAX;

    if ( _xSeries->isVisible() ) {
        auto points = _xSeries->points();

        for ( auto i : points ) {
            _minT = std::min( _minT, i.x() );
            _maxT = std::max( _maxT, i.x() );
            _minValue = std::min( _minValue, i.y() );
            _maxValue = std::max( _maxValue, i.y() );
        }

    }

    if ( _ySeries->isVisible() ) {
        auto points = _ySeries->points();

        for ( auto i : points ) {
            _minT = std::min( _minT, i.x() );
            _maxT = std::max( _maxT, i.x() );
            _minValue = std::min( _minValue, i.y() );
            _maxValue = std::max( _maxValue, i.y() );
        }

    }

    if ( _zSeries->isVisible() ) {
        auto points = _zSeries->points();

        for ( auto i : points ) {
            _minT = std::min( _minT, i.x() );
            _maxT = std::max( _maxT, i.x() );
            _minValue = std::min( _minValue, i.y() );
            _maxValue = std::max( _maxValue, i.y() );
        }

    }

    _xAxis->setRange( _minT, _maxT );
    _yAxis->setRange( _minValue, _maxValue );

}

// ImuParameters
void ImuParameters::calculate( const std::vector< XsensData > &measures )
{
    _accelMean = Eigen::Vector3d::Zero();
    _gyroMean = Eigen::Vector3d::Zero();
    _magMean = Eigen::Vector3d::Zero();

    _accelVariance = Eigen::Matrix3d::Zero();
    _gyroVariance = Eigen::Matrix3d::Zero();
    _magVariance = Eigen::Matrix3d::Zero();

    size_t size = 0;

    for ( auto &i : measures )
        if ( i.valid() ) {
            _accelMean += i.acceleration();
            _gyroMean += i.gyro();
            _magMean += i.magnitometer();

            ++size;

        }

    if ( size != 0 ) {

        _accelMean /= size;
        _gyroMean /= size;
        _magMean /= size;

        for ( auto &i : measures )
            if ( i.valid() ) {
                _accelVariance += ( _accelMean - i.acceleration() ) * ( _accelMean - i.acceleration() ).transpose();
                _gyroVariance += ( _gyroMean - i.gyro() ) * ( _gyroMean - i.gyro() ).transpose();
                _magVariance += ( _magMean - i.magnitometer() ) * ( _magMean - i.magnitometer() ).transpose();

            }

        _accelVariance /= size;
        _gyroVariance /= size;
        _magVariance /= size;

    }

}

// ImuChartWidget
ImuChartWidget::ImuChartWidget( QWidget *parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void ImuChartWidget::initialize()
{
    _accelChart = new ImuChart( "Accelerometer measurements", this );
    _gyroChart = new ImuChart( "Gyroscope measurements", this );
    _magChart = new ImuChart( "Magnitometer measurements", this );

    addWidget( _accelChart );
    addWidget( _gyroChart );
    addWidget( _magChart );
}

void ImuChartWidget::addPacket( const XsensData &packet )
{
    if ( packet.valid() ) {

        auto t = packet.xsensTime();

        auto accel = packet.acceleration();
        auto gyro = packet.gyro();
        auto mag = packet.magnitometer();

        addValue( t, accel, gyro, mag );

    }

}

void ImuChartWidget::addValue( const double t, const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro, const Eigen::Vector3d &mag )
{
    _accelChart->addValue( t, accel );
    _gyroChart->addValue( t, gyro );
    _magChart->addValue( t, mag );
}

void ImuChartWidget::clear()
{
    _accelChart->clear();
    _gyroChart->clear();
    _magChart->clear();
}

QList< QPointF > ImuChartWidget::accelXPoints() const
{
    return _accelChart->xPoints();
}

QList< QPointF > ImuChartWidget::accelYPoints() const
{
    return _accelChart->yPoints();
}

QList< QPointF > ImuChartWidget::accelZPoints() const
{
    return _accelChart->zPoints();
}

QList< QPointF > ImuChartWidget::gyroXPoints() const
{
    return _gyroChart->xPoints();
}

QList< QPointF > ImuChartWidget::gyroYPoints() const
{
    return _gyroChart->yPoints();
}

QList< QPointF > ImuChartWidget::gyroZPoints() const
{
    return _gyroChart->zPoints();
}

QList< QPointF > ImuChartWidget::magXPoints() const
{
    return _magChart->xPoints();
}

QList< QPointF > ImuChartWidget::magYPoints() const
{
    return _magChart->yPoints();
}

QList< QPointF > ImuChartWidget::magZPoints() const
{
    return _magChart->zPoints();
}

// ImuWidget
ImuWidget::ImuWidget( const QString &portName, QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize( portName );
}

void ImuWidget::initialize( const QString &portName )
{
    _rotation = Eigen::Quaterniond::Identity();
    _velocity = Eigen::Vector3d::Zero();
    _translation = Eigen::Vector3d::Zero();

    _xsensInterface.connectDevice( portName.toStdString() );
    _xsensInterface.prepare();

    _imuViewWidget = new ImuView( this );
    _chartWidget = new ImuChartWidget( this );

    addWidget( _imuViewWidget );
    addWidget( _chartWidget );

    setStretchFactor( 0, 3 );
    setStretchFactor( 1, 1 );

    startTimer( 1./20. );

}

Eigen::Quaterniond quaternion( const Eigen::Vector3d &theta )
{
    return Eigen::Quaterniond( std::cos( 0.5 * theta.x() ), std::sin( 0.5 * theta.x() ), 0, 0 )
                * Eigen::Quaterniond( std::cos( 0.5 * theta.y() ), 0, std::sin( 0.5 * theta.y() ), 0 )
                * Eigen::Quaterniond( std::cos( 0.5 * theta.z() ), 0, 0, std::sin( 0.5 * theta.z() ) );

}

void ImuWidget::timerEvent( QTimerEvent * )
{
    auto measures = _xsensInterface.getAllPackets( std::chrono::milliseconds( static_cast< int >( 1./40. ) ) );

    for ( auto &i : measures ) {

        if ( i.valid() ) {

            if ( _calibrationMeasures.size() < _calibrationSize ) {
                _calibrationMeasures.push_back( i );
                _chartWidget->addPacket( i );

            }
            else {

                if ( !_imuParameters ) {
                    _imuParameters = std::make_unique< ImuParameters >();
                    _imuParameters->calculate( _calibrationMeasures );
                    _chartWidget->clear();
                }

                if ( _prevPacket.valid() ) {

                    double dt = i.xsensTime() - _prevPacket.xsensTime() /* std::chrono::duration< double >( i.utcTime() - m_prevPacket.utcTime() ).count() */;

                    if ( dt > DBL_EPSILON ) {

                        auto gyro = 0.5 * ( _prevPacket.gyro() + i.gyro() ) - _imuParameters->_gyroMean;

                        double accScale = 0.05; // .05
                        double magScale = 0.; // .05

                        auto prevAccel = _rotation * i.acceleration();
                        auto prevMag = _rotation * i.magnitometer();

                        auto correctAccel = prevAccel.normalized();
                        auto correntMag = prevMag.normalized();

                        _rotation = _rotation * quaternion( gyro * dt * ( 1. - accScale - magScale ) )
                                            * quaternion( Eigen::Vector3d( std::asin( correctAccel.y() ), std::asin( -correctAccel.x() ), 0. ) * accScale )
                                            * quaternion( Eigen::Vector3d( 0., 0., std::asin( correntMag.y() ) ) * magScale );

                        auto worldAccel = 0.5 * ( prevAccel + _rotation * i.acceleration() );
                        auto worldMag = 0.5 * ( prevMag + _rotation * i.magnitometer() );

                        Eigen::Vector3d acceleration = worldAccel - Eigen::Vector3d( 0., 0., _imuParameters->_accelMean.norm() );

//                        if ( acceleration.norm() > 0.1 ) {
                            _translation = _translation + _velocity * dt + 0.5 * acceleration * dt * dt;
                            _velocity = _velocity + acceleration * dt;

                            std::cout << acceleration << std::endl << std::endl;
//                        }

                        _chartWidget->addValue( i.xsensTime(), acceleration, gyro, worldMag );

                        _imuViewWidget->setPose( _rotation, _translation );

                        _imuViewWidget->setAccelerationVector( worldAccel );
                        _imuViewWidget->setMagVector( worldMag );

                    }

                }

            }

            _prevPacket = i;

        }

    }

}

// ImuParametersWidget
ImuParametersWidget::ImuParametersWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}
void ImuParametersWidget::initialize()
{
    setSizePolicy( QSizePolicy::Minimum, QSizePolicy::Minimum );

    _accelMeanLabel = new QLabel( tr("Acceleration mean"), this );
    _accelMeanXLabel = new QLabel( tr("x"), this );
    _accelMeanYLabel = new QLabel( tr("y"), this );
    _accelMeanZLabel = new QLabel( tr("z"), this );
    _gyroMeanLabel = new QLabel( tr("Gyro mean"), this );
    _gyroMeanXLabel = new QLabel( tr("x"), this );
    _gyroMeanYLabel = new QLabel( tr("y"), this );
    _gyroMeanZLabel = new QLabel( tr("z"), this );
    _magMeanLabel = new QLabel( tr("Mag mean"), this );
    _magMeanXLabel = new QLabel( tr("x"), this );
    _magMeanYLabel = new QLabel( tr("y"), this );
    _magMeanZLabel = new QLabel( tr("z"), this );

    _accelVarLabel = new QLabel( tr("Acceleration variance"), this );
    _accelVarXLabel = new QLabel( tr("x"), this );
    _accelVarYLabel = new QLabel( tr("y"), this );
    _accelVarZLabel = new QLabel( tr("z"), this );
    _gyroVarLabel = new QLabel( tr("Gyro variance"), this );
    _gyroVarXLabel = new QLabel( tr("x"), this );
    _gyroVarYLabel = new QLabel( tr("y"), this );
    _gyroVarZLabel = new QLabel( tr("z"), this );
    _magVarLabel = new QLabel( tr("Mag variance"), this );
    _magVarXLabel = new QLabel( tr("x"), this );
    _magVarYLabel = new QLabel( tr("y"), this );
    _magVarZLabel = new QLabel( tr("z"), this );

    _accelMeanXLine = new QLineEdit( this );
    _accelMeanXLine->setReadOnly( true );
    _accelMeanYLine = new QLineEdit( this );
    _accelMeanYLine->setReadOnly( true );
    _accelMeanZLine = new QLineEdit( this );
    _accelMeanZLine->setReadOnly( true );
    _gyroMeanXLine = new QLineEdit( this );
    _gyroMeanXLine->setReadOnly( true );
    _gyroMeanYLine = new QLineEdit( this );
    _gyroMeanYLine->setReadOnly( true );
    _gyroMeanZLine = new QLineEdit( this );
    _gyroMeanZLine->setReadOnly( true );
    _magMeanXLine = new QLineEdit( this );
    _magMeanXLine->setReadOnly( true );
    _magMeanYLine = new QLineEdit( this );
    _magMeanYLine->setReadOnly( true );
    _magMeanZLine = new QLineEdit( this );
    _magMeanZLine->setReadOnly( true );

    _accelVarXLine = new QLineEdit( this );
    _accelVarXLine->setReadOnly( true );
    _accelVarYLine = new QLineEdit( this );
    _accelVarYLine->setReadOnly( true );
    _accelVarZLine = new QLineEdit( this );
    _accelVarZLine->setReadOnly( true );
    _gyroVarXLine = new QLineEdit( this );
    _gyroVarXLine->setReadOnly( true );
    _gyroVarYLine = new QLineEdit( this );
    _gyroVarYLine->setReadOnly( true );
    _gyroVarZLine = new QLineEdit( this );
    _gyroVarZLine->setReadOnly( true );
    _magVarXLine = new QLineEdit( this );
    _magVarXLine->setReadOnly( true );
    _magVarYLine = new QLineEdit( this );
    _magVarYLine->setReadOnly( true );
    _magVarZLine = new QLineEdit( this );
    _magVarZLine->setReadOnly( true );

    QFormLayout *layout = new QFormLayout( this );

    layout->addRow( _accelMeanLabel);
    layout->addRow( _accelMeanXLabel, _accelMeanXLine );
    layout->addRow( _accelMeanYLabel, _accelMeanYLine );
    layout->addRow( _accelMeanZLabel, _accelMeanZLine );
    layout->addRow( _gyroMeanLabel );
    layout->addRow( _gyroMeanXLabel, _gyroMeanXLine );
    layout->addRow( _gyroMeanYLabel, _gyroMeanYLine );
    layout->addRow( _gyroMeanZLabel, _gyroMeanZLine );
    layout->addRow( _magMeanLabel );
    layout->addRow( _magMeanXLabel, _magMeanXLine );
    layout->addRow( _magMeanYLabel, _magMeanYLine );
    layout->addRow( _magMeanZLabel, _magMeanZLine );

    layout->addRow( _accelVarLabel );
    layout->addRow( _accelVarXLabel, _accelVarXLine );
    layout->addRow( _accelVarYLabel, _accelVarYLine );
    layout->addRow( _accelVarZLabel, _accelVarZLine );
    layout->addRow( _gyroVarLabel );
    layout->addRow( _gyroVarXLabel, _gyroVarXLine );
    layout->addRow( _gyroVarYLabel, _gyroVarYLine );
    layout->addRow( _gyroVarZLabel, _gyroVarZLine );
    layout->addRow( _magVarLabel );
    layout->addRow( _magVarXLabel, _magVarXLine );
    layout->addRow( _magVarYLabel, _magVarYLine );
    layout->addRow( _magVarZLabel, _magVarZLine );

}

void ImuParametersWidget::setAccelMeanX( const double value )
{
    _accelMeanXLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setAccelMeanY( const double value )
{
    _accelMeanYLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setAccelMeanZ( const double value )
{
    _accelMeanZLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setGyroMeanX( const double value )
{
    _gyroMeanXLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setGyroMeanY( const double value )
{
    _gyroMeanYLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setGyroMeanZ( const double value )
{
    _gyroMeanZLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setMagMeanX( const double value )
{
    _magMeanXLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setMagMeanY( const double value )
{
    _magMeanYLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setMagMeanZ( const double value )
{
    _magMeanZLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setAccelVarX( const double value )
{
    _accelVarXLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setAccelVarY( const double value )
{
    _accelVarYLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setAccelVarZ( const double value )
{
    _accelVarZLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setGyroVarX( const double value )
{
    _gyroVarXLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setGyroVarY( const double value )
{
    _gyroVarYLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setGyroVarZ( const double value )
{
    _gyroVarZLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setMagVarX( const double value )
{
    _magVarXLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setMagVarY( const double value )
{
    _magVarYLine->setText( QString::number( value ) );
}

void ImuParametersWidget::setMagVarZ( const double value )
{
    _magVarZLine->setText( QString::number( value ) );
}

// ImuCalibrationWidget
ImuCalibrationWidget::ImuCalibrationWidget( const QString &portName, QWidget *parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize( portName );
}

void ImuCalibrationWidget::initialize( const QString &portName )
{
    _xsensInterface.connectDevice( portName.toStdString() );
    _xsensInterface.prepare();

    _chartWidget = new ImuChartWidget( this );
    _parametersWidget = new ImuParametersWidget( this );

    addWidget( _chartWidget );
    addWidget( _parametersWidget );

    setStretchFactor( 0, 1 );
    setStretchFactor( 1, 0 );

    startTimer( 1./20. );
}

void ImuCalibrationWidget::timerEvent( QTimerEvent * )
{
    auto measures = _xsensInterface.getAllPackets( std::chrono::milliseconds( static_cast< int >( 1./40. ) ) );

    for ( auto &i : measures )
        _chartWidget->addPacket( i );

    calculateParameters();

}

void ImuCalibrationWidget::calculateParameters()
{
    double mean, var;

    calculateMeanAndVariance( _chartWidget->accelXPoints(), mean, var );
    _parametersWidget->setAccelMeanX( mean );
    _parametersWidget->setAccelVarX( var );

    calculateMeanAndVariance( _chartWidget->accelYPoints(), mean, var );
    _parametersWidget->setAccelMeanY( mean );
    _parametersWidget->setAccelVarY( var );

    calculateMeanAndVariance( _chartWidget->accelZPoints(), mean, var );
    _parametersWidget->setAccelMeanZ( mean );
    _parametersWidget->setAccelVarZ( var );

    calculateMeanAndVariance( _chartWidget->gyroXPoints(), mean, var );
    _parametersWidget->setGyroMeanX( mean );
    _parametersWidget->setGyroVarX( var );

    calculateMeanAndVariance( _chartWidget->gyroYPoints(), mean, var );
    _parametersWidget->setGyroMeanY( mean );
    _parametersWidget->setGyroVarY( var );

    calculateMeanAndVariance( _chartWidget->gyroZPoints(), mean, var );
    _parametersWidget->setGyroMeanZ( mean );
    _parametersWidget->setGyroVarZ( var );

    calculateMeanAndVariance( _chartWidget->magXPoints(), mean, var );
    _parametersWidget->setMagMeanX( mean );
    _parametersWidget->setMagVarX( var );

    calculateMeanAndVariance( _chartWidget->magYPoints(), mean, var );
    _parametersWidget->setMagMeanY( mean );
    _parametersWidget->setMagVarY( var );

    calculateMeanAndVariance( _chartWidget->magZPoints(), mean, var );
    _parametersWidget->setMagMeanZ( mean );
    _parametersWidget->setMagVarZ( var );

}

void ImuCalibrationWidget::calculateMeanAndVariance( const QList< QPointF > &list, double &mean, double &var )
{
    mean = 0.;
    var = 0.;

    if ( !list.empty() ) {

        for ( auto &i : list )
           mean += i.y();

        mean /= list.size();

        for ( auto &i : list )
           var += ( mean - i.y() ) * ( mean - i.y() );

        var /= list.size();

    }

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

    _view3dWidget = new ReconstructionViewWidget( this );
    addWidget( _view3dWidget );

    int widthDiv2 = width() / 2;

    setSizes( QList< int >() << widthDiv2  << widthDiv2 );

}

void SlamViewWidget::setPath( const std::list< StereoProjectionMatrix > &path )
{
    _view3dWidget->setPath( path );
}

void SlamViewWidget::setSparseCloud(const std::vector< ColorPoint3d > &points )
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
    m_slamThread->requestInterruption();
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

    m_updateTimer = new QTimer( this );
    m_updateTimer->start( 1000 / 20 );

    connect( m_updateTimer, &QTimer::timeout, this, &SlamWidgetBase::updateViews );

    m_slamThread->start();

    updateVisibility();

}

void SlamWidgetBase::updateVisibility()
{
    m_viewWidget->showPath( m_controlWidget->isOdometryChecked() );
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
    m_viewWidget->setPath( m_slamThread->path() );
}

void SlamWidgetBase::updateSparseCloud()
{
    m_viewWidget->setSparseCloud( m_slamThread->sparseCloud() );
}

void SlamWidgetBase::updateDensePointCloud()
{
    /*auto maps = m_slamThread->maps();

    int counter = 0;

    for ( auto &map : maps ) {

        auto frames = map->frames();

        for ( auto i = frames.begin(); i != frames.end(); ++i ) {

            auto denseFrame = std::dynamic_pointer_cast< slam::FinishedDenseFrame >( *i );

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

            ++counter;*/
/*
            auto stereoFrame = std::dynamic_pointer_cast< slam::StereoFrame >( *i );
            auto processedDenseFrame = std::dynamic_pointer_cast< slam::FeatureDenseFrame >( *i );
            auto denseFrame = std::dynamic_pointer_cast< slam::ProcessedDenseFrame >( *i );

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

        /*}

    }*/

}

void SlamWidgetBase::update3dView()
{
    updatePath();
    updateSparseCloud();
    // updateDensePointCloud();
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
    m_fps = 25;

    startTimer( 1000. / m_fps );
}

void SlamImageWidget::setImageList( const QStringList &leftList, const QStringList &rightList )
{
    if ( leftList.size() == rightList.size() ) {
        m_leftList = leftList;
        m_rightList = rightList;

        m_index = 0;

    }

}

double SlamImageWidget::fps() const
{
    return m_fps;
}

void SlamImageWidget::fps( const double value )
{
    m_fps = value;
}

void SlamImageWidget::timerEvent( QTimerEvent * )
{
    static std::chrono::time_point< std::chrono::system_clock > time = std::chrono::system_clock::now();

    if ( m_index < m_leftList.size() ) {

        std::cout << m_leftList[ m_index ].toStdString() << " " << m_rightList[ m_index ].toStdString() << std::endl;

        StampedImage leftImage( time, m_leftList[ m_index ].toStdString() );
        StampedImage rightImage( time, m_rightList[ m_index ].toStdString() );

        m_slamThread->process( leftImage, rightImage );
        ++m_index;

        time += std::chrono::milliseconds{ static_cast< int64_t >( 1000.0 / m_fps  ) } ;

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
        m_slamThread->process( frame.leftImage(), frame.rightImage() );
}
