#pragma once

#include "src/common/iconswidget.h"

class QLabel;
class QBoxLayout;

class MonocularIcon;
class StereoIcon;

class CalibrationIconBase : public IconBase
{

public:
    using SuperClass = IconBase;

    CalibrationIconBase( const CvImage image, const cv::Size &frameSize, const std::vector< cv::Point3f > &worldPoints, const QString &text );

    bool isMonocularIcon() const;
    bool isStereoIcon() const;

    MonocularIcon *toMonocularIcon();
    StereoIcon *toStereoIcon();

    const MonocularIcon *toMonocularIcon() const;
    const StereoIcon *toStereoIcon() const;

    void setFrameSize( const cv::Size &size );
    const cv::Size &frameSize() const;

    void setWorldPoints( const std::vector< cv::Point3f > &points );
    const std::vector< cv::Point3f > &worldPoints() const;

protected:
    cv::Size m_frameSize;
    std::vector< cv::Point3f > m_worldPoints;

private:
    void initialize();

};

class MonocularIcon : public CalibrationIconBase
{

public:
    using SuperClass = CalibrationIconBase;

    MonocularIcon( const CvImage previewImage,
                   const cv::Size &frameSize,
                   const std::vector< cv::Point2f > &imagePoints,
                   const std::vector< cv::Point3f > &worldPoints,
                   const QString &text );

    void setImagePoints( const std::vector< cv::Point2f > &points );
    const std::vector< cv::Point2f > &imagePoints() const;

protected:
    std::vector< cv::Point2f > m_imagePoints;

private:
    void initialize();

};

class StereoIcon : public CalibrationIconBase
{

public:
    using SuperClass = CalibrationIconBase;

    StereoIcon( const CvImage leftPreviewImage,
                const CvImage rightPreviewImage,
                const cv::Size &frameSize,
                const std::vector< cv::Point2f > &leftImagePoints,
                const std::vector< cv::Point2f > &rightImagePoints,
                const std::vector< cv::Point3f > &worldPoints,
                const QString &text );

    void setLeftPreview( const CvImage &image );
    void setRightPreview( const CvImage &image );

    const CvImage &leftPreview() const;
    const CvImage &rightPreview() const;

    const CvImage stackedPreview() const;

    void setLeftImagePoints( const std::vector< cv::Point2f > &points );
    std::vector< cv::Point2f > leftImagePoints() const;

    void setRightImagePoints( const std::vector< cv::Point2f > &points );
    std::vector< cv::Point2f > rightImagePoints() const;

protected:
    CvImage m_leftPreview;
    CvImage m_rightPreview;

    std::vector< cv::Point2f > m_leftImagePoints;
    std::vector< cv::Point2f > m_rightImagePoints;

private:
    void initialize();

};

class CalibrationIconsWidget : public IconsListWidget
{
    Q_OBJECT

public:
    using SuperClass = IconsListWidget;

    explicit CalibrationIconsWidget( QWidget *parent = nullptr );

    void addIcon( CalibrationIconBase *icon );
    void insertIcon( CalibrationIconBase *icon );

    QList< CalibrationIconBase* > icons() const;

signals:
    void iconActivated( CalibrationIconBase *icon );

private:
    void initialize();

};
