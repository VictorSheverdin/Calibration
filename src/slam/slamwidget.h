#pragma once

#include <QSplitter>
#include <QTimer>
#include <QtCharts>
#include <QLineSeries>

#include <pcl/visualization/pcl_visualizer.h>

#include "slamgeometry.h"

#include "src/common/imagewidget.h"
#include "src/common/pclwidget.h"

#include "src/common/vimbacamera.h"

#include "src/common/xsens.h"

class QCheckBox;

class ImagesWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ImagesWidget( QWidget* parent = nullptr );

public slots:
    void setPointsImage( const CvImage &image );
    void setTracksImage( const CvImage &image );
    void setStereoImage( const CvImage &image );

protected:
    QPointer< ImageWidget > m_pointsWidget;
    QPointer< ImageWidget > m_tracksWidget;
    QPointer< ImageWidget > m_stereoWidget;

private:
    void initialize();

};

class ReconstructionViewWidget : public PCLWidget
{
    Q_OBJECT

public:
    explicit ReconstructionViewWidget( QWidget* parent = nullptr );

    void setPath( const std::list< StereoCameraMatrix > &path );

    void showPath( const bool value );

protected:
    vtkSmartPointer< vtkActor > m_leftTrajectoryActor;
    vtkSmartPointer< vtkActor > m_rightTrajectoryActor;

    vtkSmartPointer< vtkActor > m_leftCameraActor;
    vtkSmartPointer< vtkActor > m_rightCameraActor;

    void setLeftPath( std::list< cv::Vec3d > &points );
    void setRightPath( std::list< cv::Vec3d > &points );

    void setFrustum( const StereoCameraMatrix &cameraMatrix );
    void setLeftFrustum( const ProjectionMatrix &cameraMatrix );
    void setRightFrustum( const ProjectionMatrix &cameraMatrix );

    static void pickingEventHandler( const pcl::visualization::PointPickingEvent &event, void *viewer_void );

private:
    void initialize();

};

class ImuView : public PCLWidget
{
    Q_OBJECT

public:
    explicit ImuView( QWidget* parent = nullptr );

    static Eigen::Quaterniond alignQuaternion( const Eigen::Vector3d &value );

    void setPose( const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation );

    void setAccelerationVector( const Eigen::Vector3d &value );
    void setMagVector( const Eigen::Vector3d &value );

protected:

private:
    void initialize();

};

class ImuChart : public QWidget
{
    Q_OBJECT

public:
    explicit ImuChart( const QString &title, QWidget *parent = nullptr );

    void addValue( const double t, const Eigen::Vector3d &value );

    void clear();

    QList< QPointF > xPoints() const;
    QList< QPointF > yPoints() const;
    QList< QPointF > zPoints() const;

public slots:
    void updateVisibility();

protected:
    void updateMinMax();

    QPointer< QtCharts::QChartView > _chartView;

    QPointer< QtCharts::QChart > _chart;

    QPointer< QtCharts::QLineSeries > _xSeries;
    QPointer< QtCharts::QLineSeries > _ySeries;
    QPointer< QtCharts::QLineSeries > _zSeries;

    QPointer< QValueAxis > _xAxis;
    QPointer< QValueAxis > _yAxis;

    QPointer< QCheckBox > _xCheck;
    QPointer< QCheckBox > _yCheck;
    QPointer< QCheckBox > _zCheck;

    double _minT, _maxT;
    double _minValue, _maxValue;

    static const int _maxCount = 1e3;

private:
    void initialize( const QString &title );

};

struct ImuParameters
{
public:
    Eigen::Vector3d _accelMean;
    Eigen::Vector3d _gyroMean;
    Eigen::Vector3d _magMean;

    Eigen::Matrix3d _accelVariance;
    Eigen::Matrix3d _gyroVariance;
    Eigen::Matrix3d _magVariance;

    void calculate( const std::vector< XsensData > &measures );
};

class ImuChartWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ImuChartWidget( QWidget* parent = nullptr );

    void addPacket( const XsensData &packet );
    void addValue( const double t, const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro, const Eigen::Vector3d &mag );

    void clear();

    QList< QPointF > accelXPoints() const;
    QList< QPointF > accelYPoints() const;
    QList< QPointF > accelZPoints() const;

    QList< QPointF > gyroXPoints() const;
    QList< QPointF > gyroYPoints() const;
    QList< QPointF > gyroZPoints() const;

    QList< QPointF > magXPoints() const;
    QList< QPointF > magYPoints() const;
    QList< QPointF > magZPoints() const;

protected:
    QPointer< ImuChart > _accelChart;
    QPointer< ImuChart > _gyroChart;
    QPointer< ImuChart > _magChart;

private:
    void initialize();

};

class ImuWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ImuWidget( const QString &portName, QWidget* parent = nullptr );

protected:
    XsensInterface _xsensInterface;

    XsensData _prevPacket;

    QPointer< ImuView > _imuViewWidget;
    QPointer< ImuChartWidget > _chartWidget;

    Eigen::Quaterniond _rotation;
    Eigen::Vector3d _velocity;
    Eigen::Vector3d _translation;

    std::vector< XsensData > _calibrationMeasures;

    std::unique_ptr< ImuParameters > _imuParameters;

    static const size_t _calibrationSize = 5e2;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize( const QString &portName );

};

class ImuParametersWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImuParametersWidget( QWidget* parent = nullptr );

    void setAccelMeanX( const double value );
    void setAccelMeanY( const double value );
    void setAccelMeanZ( const double value );

    void setGyroMeanX( const double value );
    void setGyroMeanY( const double value );
    void setGyroMeanZ( const double value );

    void setMagMeanX( const double value );
    void setMagMeanY( const double value );
    void setMagMeanZ( const double value );

    void setAccelVarX( const double value );
    void setAccelVarY( const double value );
    void setAccelVarZ( const double value );

    void setGyroVarX( const double value );
    void setGyroVarY( const double value );
    void setGyroVarZ( const double value );

    void setMagVarX( const double value );
    void setMagVarY( const double value );
    void setMagVarZ( const double value );

protected:
    QPointer< QLabel > _accelMeanLabel;
    QPointer< QLabel > _accelMeanXLabel;
    QPointer< QLabel > _accelMeanYLabel;
    QPointer< QLabel > _accelMeanZLabel;
    QPointer< QLabel > _gyroMeanLabel;
    QPointer< QLabel > _gyroMeanXLabel;
    QPointer< QLabel > _gyroMeanYLabel;
    QPointer< QLabel > _gyroMeanZLabel;
    QPointer< QLabel > _magMeanLabel;
    QPointer< QLabel > _magMeanXLabel;
    QPointer< QLabel > _magMeanYLabel;
    QPointer< QLabel > _magMeanZLabel;

    QPointer< QLabel > _accelVarLabel;
    QPointer< QLabel > _accelVarXLabel;
    QPointer< QLabel > _accelVarYLabel;
    QPointer< QLabel > _accelVarZLabel;
    QPointer< QLabel > _gyroVarLabel;
    QPointer< QLabel > _gyroVarXLabel;
    QPointer< QLabel > _gyroVarYLabel;
    QPointer< QLabel > _gyroVarZLabel;
    QPointer< QLabel > _magVarLabel;
    QPointer< QLabel > _magVarXLabel;
    QPointer< QLabel > _magVarYLabel;
    QPointer< QLabel > _magVarZLabel;

    QPointer< QLineEdit > _accelMeanXLine;
    QPointer< QLineEdit > _accelMeanYLine;
    QPointer< QLineEdit > _accelMeanZLine;
    QPointer< QLineEdit > _gyroMeanXLine;
    QPointer< QLineEdit > _gyroMeanYLine;
    QPointer< QLineEdit > _gyroMeanZLine;
    QPointer< QLineEdit > _magMeanXLine;
    QPointer< QLineEdit > _magMeanYLine;
    QPointer< QLineEdit > _magMeanZLine;

    QPointer< QLineEdit > _accelVarXLine;
    QPointer< QLineEdit > _accelVarYLine;
    QPointer< QLineEdit > _accelVarZLine;
    QPointer< QLineEdit > _gyroVarXLine;
    QPointer< QLineEdit > _gyroVarYLine;
    QPointer< QLineEdit > _gyroVarZLine;
    QPointer< QLineEdit > _magVarXLine;
    QPointer< QLineEdit > _magVarYLine;
    QPointer< QLineEdit > _magVarZLine;

private:
    void initialize();

};

// TODO: Calculate IMU basic parameters
class ImuCalibrationWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit ImuCalibrationWidget( const QString &portName, QWidget* parent = nullptr );

    virtual void timerEvent( QTimerEvent * ) override;

protected:
    void calculateParameters();

    void calculateMeanAndVariance( const QList< QPointF > &list, double &mean, double &var );

    XsensInterface _xsensInterface;

    QPointer< ImuChartWidget > _chartWidget;
    QPointer< ImuParametersWidget > _parametersWidget;

private:
    void initialize( const QString &portName );

};

class SlamThread;

class SlamViewWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit SlamViewWidget( QWidget* parent = nullptr );

    void setPath( const std::list< StereoCameraMatrix > &path );
    void setSparseCloud( const std::vector< ColorPoint3d > &points );
    void setPointCloud( const std::vector< ColorPoint3d> &points, const std::string &id,
                        const Eigen::Vector4f &origin = Eigen::Vector4f::Zero(),
                        const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity() );
    void setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation );

    void showPath( const bool flag );

    bool contains( const std::string &id ) const ;

    void setPointsImage( const CvImage &image );
    void setTracksImage( const CvImage &image );
    void setStereoImage( const CvImage &image );

protected:
    QPointer< ImagesWidget > m_imagesWidget;
    QPointer< ReconstructionViewWidget > _view3dWidget;

private:
    void initialize();

};

class SlamControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SlamControlWidget( QWidget* parent = nullptr );

    const QPointer< QCheckBox > &odometryCheck() const;
    const QPointer< QCheckBox > &sparseCheck() const;
    const QPointer< QCheckBox > &denseCheck() const;

    bool isOdometryChecked() const;
    bool isSparseChecked() const;
    bool isDenseChecked() const;

protected:
    QPointer< QCheckBox > m_viewOdometryCheck;
    QPointer< QCheckBox > m_viewSparseCheck;
    QPointer< QCheckBox > m_viewDenseCheck;

private:
    void initialize();

};

class SlamWidgetBase : public QWidget
{
    Q_OBJECT

public:
    explicit SlamWidgetBase( const QString &calibrationFile, QWidget* parent = nullptr );
    ~SlamWidgetBase();

public slots:
    void updateViews();
    void updateImages();
    void update3dView();

protected slots:
    void updatePath();
    void updateSparseCloud();
    void updateDensePointCloud();

    void updateVisibility();

protected:
    QPointer< SlamControlWidget > m_controlWidget;
    QPointer< SlamViewWidget > m_viewWidget;

    QPointer< SlamThread > m_slamThread;

    QPointer< QTimer > m_updateTimer;

private:
    void initialize( const QString &calibrationFile );

};

class SlamImageWidget : public SlamWidgetBase
{
    Q_OBJECT

public:
    explicit SlamImageWidget( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, QWidget* parent = nullptr );

    void setImageList(const QStringList &leftList, const QStringList &rightList );

    double fps() const;
    void fps( const double value );

protected:
    QStringList m_leftList;
    QStringList m_rightList;

    int m_index;

    double m_fps;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize();

};

class SlamCameraWidget : public SlamWidgetBase
{
    Q_OBJECT

public:
    explicit SlamCameraWidget( const QString &leftCameraIp, const QString &rightCameraIp, const QString &calibrationFile, QWidget* parent = nullptr );

protected slots:
    void updateFrame();

protected:
    StereoCamera m_camera;

private:
    void initialize();

};
