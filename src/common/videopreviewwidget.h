#pragma once

#include <QScrollArea>
#include <QPixmap>
#include <QDockWidget>

#include "imagewidget.h"

class QLabel;
class QBoxLayout;

class VideoIconWidget : public ImageWidget
{
    Q_OBJECT

public:
    VideoIconWidget( QWidget* parent = nullptr );
    VideoIconWidget( const CvImage &icon, QWidget* parent = nullptr );

signals:
    void iconActivated( const VideoIconWidget *frame );

protected:
    virtual void paintEvent( QPaintEvent *event ) override;
    virtual void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    void initialize();
};

class VideoIconsLayout : public QWidget
{
    Q_OBJECT

public:
    explicit VideoIconsLayout( QWidget *parent = nullptr );

    void setOrientation( const Qt::Orientation value );
    Qt::Orientation orientation() const;

    void addIcon( VideoIconWidget *icon );
    void addIcons( const std::vector< VideoIconWidget* > &icons );

    QList< VideoIconWidget* > icons() const;

    unsigned int framesCount() const;

    double maximumAspectRatio() const;

    VideoIconWidget *frameAt( const size_t i ) const;

    void clear();

signals:
    void iconActivated( const VideoIconWidget *icon );

private:
    void initialize();

    QBoxLayout *m_layout;

};


class VideoIconsWidget : public QScrollArea
{
    Q_OBJECT

public:
    explicit VideoIconsWidget( QWidget *parent = nullptr );

    void setIcons( const std::vector< VideoIconWidget* > &icons );

    void addIcon( VideoIconWidget *icon );
    void addIcons( const std::vector< VideoIconWidget* > &icons );

    QList< VideoIconWidget* > icons() const;

    void setOrientation( const Qt::Orientation value );
    Qt::Orientation orientation() const;

    VideoIconsLayout *layoutWidget() const;


signals:
    void iconActivated( const VideoIconWidget* icon );

private slots:
    void updateLayout();

protected:
    virtual void resizeEvent( QResizeEvent *event ) override;

private:
    void initialize();

};
