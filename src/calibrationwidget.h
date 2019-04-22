#pragma once

#include <QSplitter>
#include <QPointer>

#include "camerawidget.h"
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

    void setSourceView( const CvImage &value );
    const CvImage &sourceView() const;

    void setProcessedView( const CvImage &value );
    const CvImage &processedView() const;

    void setRVec( const cv::Mat &value );
    const cv::Mat &rVec() const;

    void setTVec( const cv::Mat &value );
    const cv::Mat &tVec() const;

    void setPoints2d( const std::vector< cv::Point2f > &value );
    const std::vector< cv::Point2f > &points2d() const;

    void setPoints3d( const std::vector< cv::Point3f > &value );
    const std::vector< cv::Point3f > &points3d() const;

    void setOk( const bool value );
    bool isOk() const;

protected:
    CvImage m_sourceView;
    CvImage m_processedView;
    cv::Mat m_rVec;
    cv::Mat m_tVec;

    std::vector< cv::Point2f > m_points2d;
    std::vector< cv::Point3f > m_points3d;

    bool m_ok;

};

class MonocularCalibrationResults
{
public:
    MonocularCalibrationResults();

    void setFrameSize( const cv::Size &value );
    const cv::Size &frameSize() const;

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void setDistorsionCoefficients( const cv::Mat &value );
    const cv::Mat &distorsionCoefficients() const;

    void setResults( std::vector< MonocularCalibrationResult > &value );
    const std::vector< MonocularCalibrationResult > &results() const;

    MonocularCalibrationResult &result( const unsigned int i );
    const MonocularCalibrationResult &result( const unsigned int i ) const;

    void setOk( const bool value );
    bool isOk() const;

    void setError( const double value );
    double error() const;

    const unsigned int resultsSize() const;

protected:
    cv::Size m_frameSize;

    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoefficients;

    std::vector< MonocularCalibrationResult > m_results;

    bool m_ok;

    double m_error;

};

class StereoCalibrationResults
{
public:
    StereoCalibrationResults();

    void setLeftCameraResults( const MonocularCalibrationResults &value );
    const MonocularCalibrationResults &leftCameraResults() const;

    void setRightCameraResults( const MonocularCalibrationResults &value );
    const MonocularCalibrationResults &rightCameraResults() const;

    unsigned int leftResultsSize() const;
    unsigned int rightResultsSize() const;

    void setRotationMatrix( const cv::Mat &value );
    const cv::Mat &rotationMatrix() const;

    void setTranslationVector( const cv::Mat &value );
    const cv::Mat &translationVector() const;

    void setFundamentalMatrix( const cv::Mat &value );
    const cv::Mat &fundamentalMatrix() const;

    void setEssentialMatrix( const cv::Mat &value );
    const cv::Mat &essentialMatrix() const;

    void setLeftRectifyMatrix( const cv::Mat &value );
    const cv::Mat &leftRectifyMatrix() const;

    void setRightRectifyMatrix( const cv::Mat &value );
    const cv::Mat &rightRectifyMatrix() const;

    void setLeftProjectionMatrix( const cv::Mat &value );
    const cv::Mat &leftProjectionMatrix() const;

    void setRightProjectionMatrix( const cv::Mat &value );
    const cv::Mat &rightProjectionMatrix() const;

    void setDisparityToDepthMatrix( const cv::Mat &value );
    const cv::Mat &disparityToDepthMatrix() const;

    void setLeftROI( const cv::Rect &value );
    const cv::Rect &leftROI() const;

    void setRightROI( const cv::Rect &value );
    const cv::Rect &rightROI() const;

    void setLeftRMap( const cv::Mat &value );
    const cv::Mat &leftRMap() const;

    void setLeftDMap( const cv::Mat &value );
    const cv::Mat &leftDMap() const;

    void setRightRMap( const cv::Mat &value );
    const cv::Mat &rightRMap() const;

    void setRightDMap( const cv::Mat &value );
    const cv::Mat &rightDMap() const;

    void setError( const double value );
    double error() const;

    bool isOk() const;

protected:
    MonocularCalibrationResults m_leftCameraResults;
    MonocularCalibrationResults m_rightCameraResults;

    cv::Mat m_rotationMatrix;
    cv::Mat m_translationVector;
    cv::Mat m_fundamentalMatrix;
    cv::Mat m_essentialMatrix;

    cv::Mat m_leftRectifyMatrix;
    cv::Mat m_rightRectifyMatrix;
    cv::Mat m_leftProjectionMatrix;
    cv::Mat m_rightProjectionMatrix;
    cv::Mat m_disparityToDepthMatrix;

    cv::Rect m_leftROI;
    cv::Rect m_rightROI;

    cv::Mat m_leftRMap;
    cv::Mat m_leftDMap;
    cv::Mat m_rightRMap;
    cv::Mat m_rightDMap;

    double m_error;

};

class CalibrationWidgetBase : public QSplitter
{
    Q_OBJECT

public:
    CalibrationWidgetBase( QWidget *parent = nullptr );

    void setCameraDecimation( CameraWidgetBase::DecimationType type );

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

    MonocularCalibrationResults calcMonocularCalibration( const std::vector< CvImage > &frames );

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
    StereoCalibrationResults calcStereoCalibration( const std::vector< CvImage > &leftFrames,
                                                   const std::vector< CvImage > &rightFrames );

    static const int m_reportFrameSize = 400;

private:
    void initialize( const std::string &leftCameraIp, const std::string &rightCameraIp );

};
