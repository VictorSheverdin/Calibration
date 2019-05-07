#pragma once

#include <QScrollArea>
#include <QPixmap>
#include <QListWidget>

#include "src/common/imagewidget.h"

class QLabel;
class QBoxLayout;

class MonocularIcon;
class StereoIcon;

class IconBase : public QListWidgetItem
{

public:
    using SuperCalss = QListWidget;

    IconBase( const CvImage image, const int number );

    bool isMonocularIcon() const;
    bool isStereoIcon() const;

    MonocularIcon *toMonocularIcon();
    StereoIcon *toStereoIcon();

    const MonocularIcon *toMonocularIcon() const;
    const StereoIcon *toStereoIcon() const;

    const CvImage &previewImage() const;

protected:
    CvImage m_previewImage;

private:
    void initialize();

};

class MonocularIcon : public IconBase
{

public:
    using SuperCalss = IconBase;

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

class StereoIcon : public IconBase
{

public:
    using SuperCalss = IconBase;

    StereoIcon( const CvImage previewImage, const CvImage straightPreviewImage, const CvImage leftSourceImage, const CvImage rightSourceImage, const int number );

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

class IconsWidget : public QListWidget
{
    Q_OBJECT

public:
    using SuperCalss = QListWidget;

    explicit IconsWidget( QWidget *parent = nullptr );

    void addIcon( IconBase *icon );
    void insertIcon( IconBase *icon );

    QList< IconBase* > icons() const;

signals:
    void iconActivated( IconBase *icon );

protected:
    static const QSize m_iconSize;

private:
    void initialize();

};
