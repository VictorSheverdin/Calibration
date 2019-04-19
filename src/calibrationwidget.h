#pragma once

#include <QSplitter>
#include <QPointer>

#include "templateprocessor.h"

class TaskWidgetBase;
class MonocularTaskWidget;
class StereoTaskWidget;
class IconBase;
class IconsWidget;
class ImageWidget;
class ImageDialog;
class ReportDialog;

class MonocularCalibrationResult
{
public:
    MonocularCalibrationResult();

    void setFrameSize( const cv::Size &value );
    const cv::Size &frameSize() const;

    void setViews( std::vector< CvImage > &value );
    const std::vector<CvImage> &views() const;

    const CvImage &view( const unsigned int i ) const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void setDistorsionCoefficients( const cv::Mat &value );
    const cv::Mat &distorsionCoefficients() const;

    void setRVecs( const std::vector< cv::Mat > &value );
    const std::vector< cv::Mat > &rVecs() const;

    const cv::Mat &rVec( const unsigned int i ) const;

    void setTVecs( const std::vector< cv::Mat > &value );
    const std::vector< cv::Mat > &tVecs() const;

    const cv::Mat &tVec( const unsigned int i ) const;

    void setOk( const bool value );
    bool isOk() const;

    void setError( const double value );
    double error() const;

protected:
    cv::Size m_frameSize;
    std::vector< CvImage > m_views;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoefficients;

    std::vector< cv::Mat > m_rVecs;
    std::vector< cv::Mat > m_tVecs;

    bool m_ok;

    double m_error;

};

class CalibrationWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CalibrationWidgetBase( QWidget *parent = nullptr );

public slots:
    virtual void grabFrame() = 0;
    virtual void calculate() = 0;

protected slots:
    virtual void showIcon( IconBase *icon ) = 0;

protected:
    QPointer< IconsWidget > m_iconsWidget;
    QPointer< ImageDialog > m_iconViewDialog;
    QPointer< ReportDialog > m_reportDialog;

    TemplateProcessor m_processor;

    QPointer<TaskWidgetBase> m_taskWidget;

    static const int m_minimumCalibrationFrames = 5;

    MonocularCalibrationResult calcMonocularCalibration( const std::vector< CvImage > &frames );

private:
    void initialize();

};

class MonocularCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    MonocularCalibrationWidget( const std::string &cameraIp, QWidget *parent = nullptr );

    MonocularTaskWidget *taskWidget() const;

public slots:
    virtual void grabFrame() override;
    virtual void calculate() override;

protected slots:
    virtual void showIcon( IconBase *icon ) override;

protected:

private:
    void initialize( const std::string &cameraIp );

};

class StereoCalibrationWidget : public CalibrationWidgetBase
{
    Q_OBJECT

public:
    StereoCalibrationWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent = nullptr );

    void saveXMLCalibration();

    StereoTaskWidget *taskWidget() const;

protected slots:
    virtual void showIcon( IconBase *icon ) override;

public slots:
    virtual void grabFrame() override;
    virtual void calculate() override;

protected:
    QPointer< StereoTaskWidget > m_taskWidget;

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};
