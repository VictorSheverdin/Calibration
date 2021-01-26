#pragma once

#include <QWidget>
#include <QPointer>
#include <QScrollArea>

#include "image.h"

#include "supportwidgets.h"

class ImageWidget : public QWidget
{
    Q_OBJECT

public:
    ImageWidget( QWidget* parent = nullptr );
    ImageWidget( const CvImage frame, QWidget* parent = nullptr );

    const CvImage image() const;

    int imageWidth() const;
    int imageHeight() const;
    double imageAspect() const;


signals:
    void resizeSignal();

public slots:
    void setImage( const CvImage &image );

protected:
    CvImage m_image;

    virtual void paintEvent( QPaintEvent *event ) override;
    virtual void resizeEvent( QResizeEvent *event ) override;

private:
    void initialize();

};

class ImageViewer : public QScrollArea
{
    Q_OBJECT

 public:
     ImageViewer( QWidget* parent = nullptr );

public slots:
     void setImage( const CvImage &image );

     void setScale( const double value );

     void zoomIn();
     void zoomOut();
     void normalSize();

protected:
    QPointer< ImageWidget > m_image;
    double m_scaleFactor;

    void wheelEvent( QWheelEvent *event ) override;

    void scaleImage( double factor );

    void updateScale();

private:
    void initialize();

};

class ImageDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit ImageDialog( QWidget *parent = nullptr );

    ImageWidget *widget() const;

public slots:
    void setImage(const CvImage image);

protected:

private:
    void initialize();

};
