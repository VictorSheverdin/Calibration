#pragma once

#include <QSplitter>
#include <QTimer>

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

class ImuViewWidget : public PCLWidget
{
    Q_OBJECT

public:
    explicit ImuViewWidget( const QString &portName, QWidget* parent = nullptr );

    static Eigen::Quaterniond alignQuaternion( const Eigen::Vector3d &value );

    void setRotation( const Eigen::Quaterniond &value );
    const Eigen::Quaterniond &rotation() const;

protected:
    XsensInterface m_xsensInterface;

    Eigen::Quaterniond m_rotation;

    XsensData m_prevPacket;

    virtual void timerEvent( QTimerEvent * ) override;

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
    void setSparseCloud( const std::list< ColorPoint3d > &points );
    void setPointCloud( const std::list< ColorPoint3d > &points, const std::string &id,
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
