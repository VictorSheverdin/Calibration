#pragma once

#include <QSplitter>
#include <QPointer>

#include "src/common/templateprocessor.h"
#include "src/common/markerprocessor.h"

#include "reportwidget.h"
#include "src/common/defs.h"

#include "threads.h"

class GrabWidgetBase;
class MonocularGrabWidget;
class StereoGrabWidget;
class CalibrationIconBase;
class CalibrationIconsWidget;
class MonocularIcon;
class StereoIcon;
class ImageWidget;
class ImageDialog;
class ParametersWidget;

class CalibrationWidgetBase : public QWidget
{
    Q_OBJECT

public slots:
    virtual void importDialog() = 0;
    virtual void exportDialog() = 0;

    virtual void calculate() = 0;

    void clearIcons();

protected slots:
    virtual void showIcon( CalibrationIconBase *icon ) = 0;

protected:
    CalibrationWidgetBase( QWidget *parent = nullptr );

    QPointer< QVBoxLayout > m_layout;
    QPointer< CalibrationIconsWidget > m_iconsList;
    QPointer< ImageDialog > m_iconViewDialog;

    static const int m_minimumCalibrationPoints = 7;
    static const int m_minimumCalibrationFrames = 5;

    MonocularCalibrationData calcMonocularCalibration( const QList<CalibrationIconBase *> &icons );
    MonocularCalibrationData calcMonocularCalibration( const std::vector< std::vector< cv::Point2f > > &points2d, const std::vector<std::vector<cv::Point3f> > &points3d, cv::Size &frameSize );

    StereoCalibrationData calcStereoCalibration( const QList<CalibrationIconBase *> &icons );
    StereoCalibrationData calcStereoCalibration( const std::vector< std::vector< cv::Point2f > > &leftPoints, const std::vector< std::vector< cv::Point2f > > &rightPoints, const std::vector<std::vector<cv::Point3f> > &points3d, cv::Size &frameSize );

    int m_iconCount;

    void dropIconCount();

    void addIcon( CalibrationIconBase *icon );
    void insertIcon( CalibrationIconBase *icon );

private:
    void initialize();

};

class MonocularCalibrationWidgetBase : public CalibrationWidgetBase
{
    Q_OBJECT

protected slots:
    virtual void showIcon( CalibrationIconBase *icon ) override;

protected:
    MonocularCalibrationWidgetBase( QWidget *parent = nullptr );

    MonocularProcessorThread m_processorThread;

private:
    void initialize();

};

class StereoCalibrationWidgetBase : public CalibrationWidgetBase
{
    Q_OBJECT

protected slots:
    virtual void showIcon( CalibrationIconBase *icon ) override;

protected:
    StereoCalibrationWidgetBase( QWidget *parent = nullptr );

    StereoProcessorThread m_processorThread;

private:
    void initialize();

};

class MonocularImageCalibrationWidget : public MonocularCalibrationWidgetBase
{
    Q_OBJECT

public:
    MonocularImageCalibrationWidget( QWidget *parent = nullptr );

public slots:
    virtual void importDialog() override;
    virtual void exportDialog() override;

    virtual void calculate() override;

    void addIcon( const CvImage &image );
    void insertIcon( const CvImage &image );

    void loadIcon( const QString &fileName );

protected:
    QPointer< ParametersWidget > m_parametersWidget;

    MonocularIcon *createIcon( const CvImage &image );

private:
    void initialize();
};

class StereoImageCalibrationWidget : public StereoCalibrationWidgetBase
{
    Q_OBJECT

public:
    StereoImageCalibrationWidget( QWidget *parent = nullptr );

public slots:
    virtual void importDialog() override;
    virtual void exportDialog() override;

    virtual void calculate() override;

    void addIcon( const CvImage &leftImage, const CvImage &rightImage );
    void insertIcon( const CvImage &leftImage, const CvImage &rightImage );

    void loadIcon( const QString &leftFileName, const QString &rightFileName );

protected:
    QPointer< ParametersWidget > m_parametersWidget;

    StereoIcon *createIcon( const CvImage &leftImage, const CvImage &rightImage );

private:
    void initialize();
};

class CameraCalibrationWidgetBase
{
public:
    CameraCalibrationWidgetBase();

    virtual void grabFrame() = 0;

private:
    void initialize();

};

class MonocularCameraCalibrationWidget : public MonocularCalibrationWidgetBase, public CameraCalibrationWidgetBase
{
    Q_OBJECT

public:
    MonocularCameraCalibrationWidget( const QString &cameraIp, QWidget *parent = nullptr );

    MonocularGrabWidget *taskWidget() const;

public slots:
    virtual void importDialog() override;
    virtual void exportDialog() override;

    virtual void grabFrame() override;
    virtual void calculate() override;

    void addIcon( const CvImage &image );
    void insertIcon( const CvImage &image );

    void loadIcon( const QString &fileName );

protected slots:
    void makeIcon();

protected:
    QPointer< QSplitter > m_splitter;
    QPointer< MonocularGrabWidget > m_taskWidget;

    MonocularIcon *createIcon( const CvImage &image );

private:
    void initialize( const QString &cameraIp );

};

class StereoCameraCalibrationWidget : public StereoCalibrationWidgetBase, public CameraCalibrationWidgetBase
{
    Q_OBJECT

public:
    StereoCameraCalibrationWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent = nullptr );

    StereoGrabWidget *taskWidget() const;

public slots:
    virtual void importDialog() override;
    virtual void exportDialog() override;

    virtual void grabFrame() override;
    virtual void calculate() override;

    void addIcon( const CvImage &leftImage, const CvImage &rightImage );
    void insertIcon( const CvImage &leftImage, const CvImage &rightImage );

    void loadIcon( const QString &leftFileName, const QString &rightFileName );

protected slots:
    void makeIcon();

protected:
    QPointer< QSplitter > m_splitter;
    QPointer< StereoGrabWidget > m_taskWidget;

    StereoIcon *createIcon( const CvImage &leftImage, const CvImage &rightImage );

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};
