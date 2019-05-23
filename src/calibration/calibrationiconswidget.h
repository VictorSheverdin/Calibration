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

    CalibrationIconBase( const CvImage image, const int number );

    bool isMonocularIcon() const;
    bool isStereoIcon() const;

    MonocularIcon *toMonocularIcon();
    StereoIcon *toStereoIcon();

    const MonocularIcon *toMonocularIcon() const;
    const StereoIcon *toStereoIcon() const;

private:
    void initialize();

};

class MonocularIcon : public CalibrationIconBase
{

public:
    using SuperClass = CalibrationIconBase;

    MonocularIcon( const CvImage previewImage, const CvImage sourceImage, const int number );

    void setSourceImage( const CvImage &image );
    const CvImage sourceImage() const;

    void setPreviewPoints( std::vector< cv::Point2f > &points );
    std::vector< cv::Point2f > previewPoints() const;

protected:
    std::vector< cv::Point2f > m_previewPoints;
    CvImage m_sourceImage;

private:
    void initialize();

};

class StereoIcon : public CalibrationIconBase
{

public:
    using SuperClass = CalibrationIconBase;

    StereoIcon( const CvImage leftPreviewImage, const CvImage rightPreviewImage, const CvImage leftSourceImage, const CvImage rightSourceImage, const int number );

    void setStraightPreview(const CvImage &image);
    void setLeftSourceImage( const CvImage &image );
    void setRightSourceImage( const CvImage &image );

    const CvImage &straightPreview() const;
    const CvImage leftSourceImage() const;
    const CvImage rightSourceImage() const;

    void setLeftPreviewPoints( std::vector< cv::Point2f > &points );
    std::vector< cv::Point2f > leftPreviewPoints() const;

    void setRightPreviewPoints( std::vector< cv::Point2f > &points );
    std::vector< cv::Point2f > rightPreviewPoints() const;

protected:
    CvImage m_straightPreview;

    std::vector<cv::Point2f> m_previewLeftPoints;
    std::vector<cv::Point2f> m_previewRightPoints;

    CvImage m_leftSourceImage;
    CvImage m_rightSourceImage;

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
