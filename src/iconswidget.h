#pragma once

#include <QScrollArea>
#include <QPixmap>

#include "imagewidget.h"

class QLabel;
class QBoxLayout;

class IconData
{
public:
    IconData();

};

class MonocularIcon;
class StereoIcon;

class IconBase : public ImageWidget
{
    Q_OBJECT

public:
    IconBase( QWidget* parent = nullptr );
    IconBase( const CvImage frame, QWidget* parent = nullptr );

    void setPreviewImage( const CvImage &image );
    const CvImage previewImage() const;

    bool isMonocularIcon() const;
    bool isStereoIcon() const;

    MonocularIcon *toMonocularIcon();
    StereoIcon *toStereoIcon();

    const MonocularIcon *toMonocularIcon() const;
    const StereoIcon *toStereoIcon() const;

protected:
    virtual void paintEvent( QPaintEvent *event ) override;
    virtual void mouseDoubleClickEvent(QMouseEvent *event) override;

signals:
    void iconActivated( IconBase *icon );

private:
    void initialize();

};

class MonocularIcon : public IconBase
{
    Q_OBJECT

public:
    MonocularIcon( const CvImage previewImage, const CvImage sourceImage, QWidget* parent = nullptr );

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
    Q_OBJECT

public:
    StereoIcon( const CvImage previewImage, const CvImage leftSourceImage, const CvImage rightSourceImage, QWidget* parent = nullptr );

    void setLeftSourceImage( const CvImage &image );
    void setRightSourceImage( const CvImage &image );

    const CvImage leftSourceImage() const;
    const CvImage rightSourceImage() const;

    void setLeftPreviewPoints( std::vector< cv::Point2f > &points );
    std::vector< cv::Point2f > leftPreviewPoints() const;

    void setRightPreviewPoints( std::vector< cv::Point2f > &points );
    std::vector< cv::Point2f > rightPreviewPoints() const;

protected:
    std::vector<cv::Point2f> m_previewLeftPoints;
    std::vector<cv::Point2f> m_previewRightPoints;

    CvImage m_leftSourceImage;
    CvImage m_rightSourceImage;

private:
    void initialize();

};

class IconsLayout : public QWidget
{
    Q_OBJECT

public:
    explicit IconsLayout( QWidget *parent = nullptr );

    void setOrientation( const Qt::Orientation value );
    Qt::Orientation orientation() const;

    void insertIcon( IconBase *icon );
    void addIcon( IconBase *icon );

    unsigned int iconsCount() const;

    double maximumAspectRatio() const;

    QList< IconBase* > icons() const;
    IconBase *iconAt( const size_t i ) const;

    void clear();

signals:
    void iconActivated( IconBase *icon );

private:
    void initialize();

    QBoxLayout *m_layout;

};


class IconsWidget : public QScrollArea
{
    Q_OBJECT

public:
    explicit IconsWidget( QWidget *parent = nullptr );

    void addIcon( IconBase *icon );
    void insertIcon( IconBase *icon );

    void setOrientation( const Qt::Orientation value );
    Qt::Orientation orientation() const;

    IconsLayout *layoutWidget() const;

    QList< IconBase* > icons() const;

signals:
    void iconActivated( IconBase *icon );

private slots:
    void updateLayout();

protected:
    virtual void resizeEvent(QResizeEvent *event) override;

private:
    void initialize();

};


