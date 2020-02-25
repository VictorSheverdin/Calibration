#include "src/common/precompiled.h"

#include "previewwidget.h"

CameraPreviewWidget::CameraPreviewWidget( QWidget *parent )
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

void CameraPreviewWidget::setTemplateExist( const bool value )
{
    m_templateExist = value;
}

bool CameraPreviewWidget::isTemplateExist() const
{
    return m_templateExist;
}

void CameraPreviewWidget::setSourceImage( const CvImage image )
{
    m_sourceImage = image;
}

void CameraPreviewWidget::setPreviewImage( const CvImage image )
{
    ImageWidget::setImage( image );
}
