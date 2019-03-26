#pragma once

#include <QWidget>

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

