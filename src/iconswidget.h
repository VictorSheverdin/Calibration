#pragma once

#include <QScrollArea>
#include <QPixmap>

#include "imagewidget.h"

class QLabel;
class QBoxLayout;

class IconBase : public ImageWidget
{
    Q_OBJECT

public:
    IconBase( QWidget* parent = nullptr );
    IconBase( const CvImage frame, QWidget* parent = nullptr );

    void setPreviewImage( const CvImage &image );
    const CvImage previewImage() const;

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

protected:
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

protected:
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

signals:
    void iconActivated( IconBase *icon );

private slots:
    void updateLayout();

protected:
    virtual void resizeEvent(QResizeEvent *event) override;

private:
    void initialize();

};


