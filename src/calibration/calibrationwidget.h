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

class CalibrationWidgetBase : public QWidget
{
    Q_OBJECT

public slots:
    virtual void importDialog() = 0;
    virtual void exportDialog() = 0;

    virtual void calculate() = 0;

    void clearIcons();

protected slots:
    virtual void showIcon( IconBase *icon ) = 0;

protected:
    CalibrationWidgetBase( QWidget *parent = nullptr );

    QPointer< QVBoxLayout > m_layout;
    QPointer< IconsWidget > m_iconsList;
    QPointer< ImageDialog > m_iconViewDialog;

    TemplateProcessor m_processor;

    static const int m_minimumCalibrationFrames = 5;

    MonocularCalibrationData calcMonocularCalibration( const std::vector< CvImage > &frames, const cv::Size &count, const double size );

    StereoCalibrationData calcStereoCalibration( const std::vector< CvImage > &leftFrames, const std::vector< CvImage > &rightFrames,
                                                 const cv::Size &count, const double size );

    int m_iconCount;

    void dropIconCount();

    void addIcon( IconBase *icon );
    void insertIcon( IconBase *icon );

private:
    void initialize();

};

class MonocularCalibrationWidgetBase : public CalibrationWidgetBase
{
    Q_OBJECT

protected slots:
    virtual void showIcon( IconBase *icon ) override;

protected:
    MonocularCalibrationWidgetBase( QWidget *parent = nullptr );

    QPointer< MonocularReportDialog > m_reportDialog;

private:
    void initialize();

};

class StereoCalibrationWidgetBase : public CalibrationWidgetBase
{
    Q_OBJECT

protected slots:
    virtual void showIcon( IconBase *icon ) override;

protected:
    StereoCalibrationWidgetBase( QWidget *parent = nullptr );

    QPointer< StereoReportDialog > m_reportDialog;

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

protected:
    QPointer< QSplitter > m_splitter;
    QPointer< StereoGrabWidget > m_taskWidget;

    StereoIcon *createIcon( const CvImage &leftImage, const CvImage &rightImage );

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};
