#include "src/common/precompiled.h"

#include "previewwidget.h"

CameraPreviewWidget::CameraPreviewWidget( QWidget* parent )
    : ImageWidget(parent)
{
    initialize();
}

void CameraPreviewWidget::initialize()
{
}

const CvImage CameraPreviewWidget::sourceImage() const
{
    return m_sourceImage;
}

const CvImage CameraPreviewWidget::previewImage() const
{
    return ImageWidget::image();
}

const std::vector<cv::Point2f> &CameraPreviewWidget::previewPoints() const
{
    return m_previewPoints;
}

bool CameraPreviewWidget::isTemplateExist() const
{
    return !previewPoints().empty();
}

void CameraPreviewWidget::setSourceImage(const CvImage image)
{
    m_sourceImage = image;
}

void CameraPreviewWidget::setPreviewImage(const CvImage image)
{
    ImageWidget::setImage( image );
}

void CameraPreviewWidget::setPreviewPoints( const std::vector<cv::Point2f> &points )
{
    m_previewPoints = points;
}
