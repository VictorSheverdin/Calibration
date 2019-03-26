#pragma once

#include <QThread>
#include <QSplitter>

#include "imagewidget.h"
#include "templateprocessor.h"

#include <opencv2/opencv.hpp>

class PreviewWidget : public ImageWidget
{
    Q_OBJECT

public:
    PreviewWidget( QWidget* parent = nullptr );

    const CvImage sourceImage() const;
    const CvImage previewImage() const;

public slots:
    void setSourceImage(const CvImage image);
    void setDisplayedImage(const CvImage image);

protected:
    CvImage m_sourceImage;

private:
    void initialize();

};
