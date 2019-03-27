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

const std::vector<cv::Point2f> &PreviewWidget::previewPoints() const
{
    return m_previewPoints;
}

void PreviewWidget::setSourceImage(const CvImage image)
{
    m_sourceImage = image;
}

void PreviewWidget::setPreviewImage(const CvImage image)
{
    ImageWidget::setImage( image );
}

void PreviewWidget::setPreviewPoints( const std::vector<cv::Point2f> &points )
{
    m_previewPoints = points;
}
