#include "src/common/precompiled.h"

#include "taskwidget.h"

#include "parameterswidget.h"
#include "camerawidget.h"

// GrabWidgetBase
GrabWidgetBase::GrabWidgetBase( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void GrabWidgetBase::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );

    m_layout = new QVBoxLayout( this );

    m_parametersWidget = new CameraParametersWidget( this );

    m_layout->addWidget( m_parametersWidget );

    connect (m_parametersWidget, &CameraParametersWidget::parametersChanges, this, &GrabWidgetBase::updateParameters );

}

TypeComboBox::Type GrabWidgetBase::templateType() const
{
    return m_cameraWidget->templateType();
}

const cv::Size &GrabWidgetBase::templateCount() const
{
    return m_cameraWidget->templateCount();
}

double GrabWidgetBase::templateSize() const
{
    return m_cameraWidget->templateSize();
}

bool GrabWidgetBase::resizeFlag() const
{
    return m_cameraWidget->resizeFlag();
}

unsigned int GrabWidgetBase::frameMaximumFlag() const
{
    return m_cameraWidget->frameMaximumFlag();
}

bool GrabWidgetBase::adaptiveThreshold() const
{
    return m_cameraWidget->adaptiveThreshold();
}

bool GrabWidgetBase::normalizeImage() const
{
    return m_cameraWidget->normalizeImage();
}

bool GrabWidgetBase::filterQuads() const
{
    return m_cameraWidget->filterQuads();
}

bool GrabWidgetBase::fastCheck() const
{
    return m_cameraWidget->fastCheck();
}

void GrabWidgetBase::updateParameters()
{
    m_cameraWidget->setType( m_parametersWidget->templateType() );
    m_cameraWidget->setCount( m_parametersWidget->templateCount() );
    m_cameraWidget->setTemplateSize( m_parametersWidget->templateSize() );

    m_cameraWidget->setAdaptiveThreshold( m_parametersWidget->adaptiveThreshold() );
    m_cameraWidget->setNormalizeImage( m_parametersWidget->normalizeImage() );
    m_cameraWidget->setFilterQuads( m_parametersWidget->filterQuads() );
    m_cameraWidget->setFastCheck( m_parametersWidget->fastCheck() );

    m_cameraWidget->setResizeFlag( m_parametersWidget->rescaleFlag() );
    m_cameraWidget->setFrameMaximumSize( m_parametersWidget->rescaleSize() );

}

// MonocularGrabWidget
MonocularGrabWidget::MonocularGrabWidget( const QString &cameraIp, QWidget* parent )
    : GrabWidgetBase( parent )
{
    initialize( cameraIp );
}

void MonocularGrabWidget::initialize( const QString &cameraIp )
{
    m_cameraWidget = new MonocularCameraWidget( cameraIp, this );
    m_layout->addWidget( m_cameraWidget );

    updateParameters();
}

MonocularCameraWidget *MonocularGrabWidget::cameraWidget() const
{
    return dynamic_cast< MonocularCameraWidget * >( m_cameraWidget.data() );
}

const CvImage MonocularGrabWidget::sourceImage() const
{
    return cameraWidget()->sourceImage();
}

const CvImage MonocularGrabWidget::previewImage() const
{
    return cameraWidget()->previewImage();
}

bool MonocularGrabWidget::isTemplateExist() const
{
    return cameraWidget()->isTemplateExist();
}

// StereoGrabWidget
StereoGrabWidget::StereoGrabWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : GrabWidgetBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoGrabWidget::initialize( const QString &leftCameraIp, const QString &rightCameraIp )
{
    m_cameraWidget = new StereoCameraWidget( leftCameraIp,  rightCameraIp, this );
    m_layout->addWidget( m_cameraWidget );

    updateParameters();
}

StereoCameraWidget *StereoGrabWidget::cameraWidget() const
{
    return dynamic_cast< StereoCameraWidget * >( m_cameraWidget.data() );
}

const CvImage StereoGrabWidget::leftDisplayedImage() const
{
    return cameraWidget()->leftDisplayedImage();
}

const CvImage StereoGrabWidget::rightDisplayedImage() const
{
    return cameraWidget()->rightDisplayedImage();
}

bool StereoGrabWidget::isTemplateExist() const
{
    return cameraWidget()->isTemplateExist();
}

const StereoFrame StereoGrabWidget::frame() const
{
    return cameraWidget()->stereoFrame();
}

void StereoGrabWidget::setLeftSourceImage( const CvImage image )
{
    return cameraWidget()->setLeftSourceImage( image );
}

void StereoGrabWidget::setLeftDisplayedImage( const CvImage image )
{
    return cameraWidget()->setLeftDisplayedImage( image );
}

void StereoGrabWidget::setRightSourceImage( const CvImage image )
{
    return cameraWidget()->setRightSourceImage( image );
}

void StereoGrabWidget::setRightDisplayedImage( const CvImage image )
{
    return cameraWidget()->setRightDisplayedImage( image );
}
