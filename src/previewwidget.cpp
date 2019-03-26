#include "precompiled.h"

#include "previewwidget.h"

PreviewWidget::PreviewWidget( QWidget* parent )
    : ImageWidget(parent)
{
    initialize();
}

void PreviewWidget::initialize()
{
}

const CvImage PreviewWidget::sourceImage() const
{
    return m_sourceImage;
}

const CvImage PreviewWidget::previewImage() const
{
    return ImageWidget::image();
}

void PreviewWidget::setSourceImage(const CvImage image)
{
    m_sourceImage = image;
}

void PreviewWidget::setDisplayedImage(const CvImage image)
{
    ImageWidget::setImage( image );
}
