#pragma once

#include <QWidget>
#include <QDialog>
#include <QPointer>

#include "image.h"

class ImageWidget : public QWidget
{
    Q_OBJECT

public:
    ImageWidget( QWidget* parent = nullptr );
    ImageWidget(const CvImage frame, QWidget* parent = nullptr );

    const CvImage image() const;

    int imageWidth() const;
    int imageHeight() const;
    double imageAspect() const;


signals:
    void resizeSignal();

public slots:
    void setImage(const CvImage image);

protected:
    CvImage m_image;

    virtual void paintEvent( QPaintEvent *event ) override;
    virtual void resizeEvent(QResizeEvent *event) override;

private:
    void initialize();

};

class ImageDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ImageDialog( QWidget *parent = nullptr );

public slots:
    void setImage(const CvImage image);

protected:
    QPointer< ImageWidget > m_imageWidget;

private:
    void initialize();

};
