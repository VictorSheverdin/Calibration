#pragma once

#include <QSplitter>
#include <QPointer>

#include "templateprocessor.h"

#include "reportwidget.h"
#include "src/common/defs.h"

class GrabWidgetBase;
class MonocularGrabWidget;
class StereoGrabWidget;
class IconBase;
class IconsWidget;
class MonocularIcon;
class StereoIcon;
class ImageWidget;
class ImageDialog;
class ParametersWidget;


class CalibrationWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CalibrationWidgetBase( QWidget *parent = nullptr );

public slots:
    virtual void importDialog() = 0;
    virtual void exportDialog() = 0;

    virtual void calculate() = 0;

    void clearIcons();

protected slots:
    virtual void showIcon( IconBase *icon ) = 0;

protected:
    QPointer< IconsWidget > m_iconsList;
    QPointer< ImageDialog > m_iconViewDialog;

    TemplateProcessor m_processor;

    static const int m_minimumCalibrationFrames = 5;

    MonocularCalibrationData calcMonocularCalibration( const std::vector< CvImage > &frames , const cv::Size &count, const double size );

    int m_iconCount;

    void dropIconCount();

    void addIcon( IconBase *icon );
    void insertIcon( IconBase *icon );

private:
    void initialize();

};

class MonocularCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    MonocularCalibrationWidget( QWidget *parent = nullptr );

public slots:
    virtual void importDialog() override;
    virtual void exportDialog() override;

    virtual void calculate() override;

protected slots:
    virtual void showIcon( IconBase *icon ) override;

    void addIcon( const CvImage &image );
    void insertIcon( const CvImage &image );

    void loadIcon( const QString &fileName );

protected:
    QPointer< ParametersWidget > m_parametersWidget;

    MonocularIcon *createIcon( const CvImage &image );

private:
    void initialize();

};

class StereoCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    StereoCalibrationWidget( QWidget *parent = nullptr );

public slots:
    virtual void importDialog() override;
    virtual void exportDialog() override;

    virtual void calculate() override;

    void addIcon( const CvImage &leftImage, const CvImage &rightImage );
    void insertIcon( const CvImage &leftImage, const CvImage &rightImage );

    void loadIcon( const QString &leftFileName, const QString &rightFileName );

protected slots:
    virtual void showIcon( IconBase *icon ) override;

protected:
    QPointer< ParametersWidget > m_parametersWidget;

    StereoIcon *createIcon( const CvImage &leftImage, const CvImage &rightImage );

private:
    void initialize();

};

class CameraCalibrationWidgetBase : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    CameraCalibrationWidgetBase( QWidget *parent = nullptr );

public slots:
    virtual void grabFrame() = 0;

private:
    void initialize();

};

class MonocularCameraCalibrationWidget : public CameraCalibrationWidgetBase
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

protected slots:
    virtual void showIcon( IconBase *icon ) override;

protected:
    QPointer< MonocularGrabWidget > m_taskWidget;

    QPointer< MonocularReportDialog > m_reportDialog;

private:
    void initialize( const QString &cameraIp );

};

class StereoCameraCalibrationWidget : public CameraCalibrationWidgetBase
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

protected slots:
    virtual void showIcon( IconBase *icon ) override;

protected:
    StereoCalibrationData calcStereoCalibration( const std::vector< CvImage > &leftFrames,
                                                   const std::vector< CvImage > &rightFrames );

    QPointer< StereoGrabWidget > m_taskWidget;

    QPointer< StereoReportDialog > m_reportDialog;

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};
