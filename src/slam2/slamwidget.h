#pragma once

#include <QSplitter>
#include <QTimer>

#include <pcl/visualization/pcl_visualizer.h>

#include "src/common/imagewidget.h"
#include "src/common/pclwidget.h"

#include "src/common/projectionmatrix.h"

class QCheckBox;
class ProcessorThread;

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
    QPointer< ImageWidget > _pointsWidget;
    QPointer< ImageWidget > _tracksWidget;
    QPointer< ImageWidget > _stereoWidget;

private:
    void initialize();

};

class ReconstructionViewWidget : public PCLWidget
{
    Q_OBJECT

public:
    explicit ReconstructionViewWidget( QWidget* parent = nullptr );

    void setPath( const std::vector< StereoProjectionMatrix > &points );
    void setFrustum( const StereoProjectionMatrix &cameraMatrix );

    void showPath( const bool value );
    void showFrustum( const bool value );

protected:
    vtkSmartPointer< vtkActor > _leftTrajectoryActor;
    vtkSmartPointer< vtkActor > _rightTrajectoryActor;

    vtkSmartPointer< vtkActor > _leftCameraActor;
    vtkSmartPointer< vtkActor > _rightCameraActor;

    static void pickingEventHandler( const pcl::visualization::PointPickingEvent &event, void *viewer_void );

    void setLeftPath( const std::vector< cv::Point3d > &points );
    void setRightPath( const std::vector< cv::Point3d > &points );

    void setLeftFrustum( const ProjectionMatrix &cameraMatrix );
    void setRightFrustum( const ProjectionMatrix &cameraMatrix );

private:
    void initialize();

};

class SlamViewWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit SlamViewWidget( QWidget* parent = nullptr );

    void setPath( const std::vector< StereoProjectionMatrix > &path );
    void setCamera(const StereoProjectionMatrix &camera );

    void setSparseCloud( const std::vector< ColorPoint3d > &points );
    void setPointCloud( const std::vector< ColorPoint3d > &points, const std::string &id,
                        const Eigen::Vector4f &origin = Eigen::Vector4f::Zero(),
                        const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity() );
    void setPointCloudPose( const std::string &id, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation );

    void showPath( const bool flag );
    void showFrustum( const bool flag );
    void showSparseCloud( const bool flag );

    bool contains( const std::string &id ) const ;

    void setPointsImage( const CvImage &image );
    void setTracksImage( const CvImage &image );
    void setStereoImage( const CvImage &image );

protected:
    QPointer< ImagesWidget > _imagesWidget;
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
    QPointer< QCheckBox > _viewOdometryCheck;
    QPointer< QCheckBox > _viewSparseCheck;
    QPointer< QCheckBox > _viewDenseCheck;

private:
    void initialize();

};

class SlamWidgetBase : public QWidget
{
    Q_OBJECT

public:
    explicit SlamWidgetBase( const QString &calibrationFile, const QString &leftMaskFile, const QString &rightMaskFile, QWidget* parent = nullptr );
    ~SlamWidgetBase();

public slots:
    void updateViews();
    void updateImages();
    void update3DView();

protected slots:
    void updateVisibility();

protected:
    QPointer< SlamControlWidget > _controlWidget;
    QPointer< SlamViewWidget > _viewWidget;

    QPointer< ProcessorThread > _processorThread;

    QPointer< QTimer > _updateTimer;

private:
    void initialize( const QString &calibrationFile, const QString &leftMaskFile, const QString &rightMaskFile );

};

class SlamImageWidget : public SlamWidgetBase
{
    Q_OBJECT

public:
    explicit SlamImageWidget( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, const QString &leftMaskFile, const QString &rightMaskFile, QWidget* parent = nullptr );

    void setImageList(const QStringList &leftList, const QStringList &rightList );

    double fps() const;
    void fps( const double value );

protected:
    QStringList _leftList;
    QStringList _rightList;

    int _index;

    double _fps;

    virtual void timerEvent( QTimerEvent * ) override;

private:
    void initialize();

};
