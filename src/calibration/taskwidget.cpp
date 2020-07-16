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

}

TypeComboBox::Type GrabWidgetBase::templateType() const
{
    return m_parametersWidget->templateType();
}

const cv::Size GrabWidgetBase::templateCount() const
{
    return m_parametersWidget->templateCount();
}

double GrabWidgetBase::templateSize() const
{
    return m_parametersWidget->templateSize();
}

bool GrabWidgetBase::resizeFlag() const
{
    return m_parametersWidget->rescaleFlag();
}

unsigned int GrabWidgetBase::frameMaximumSize() const
{
    return m_parametersWidget->rescaleSize();
}

bool GrabWidgetBase::adaptiveThreshold() const
{
    return m_parametersWidget->adaptiveThreshold();
}

bool GrabWidgetBase::normalizeImage() const
{
    return m_parametersWidget->normalizeImage();
}

bool GrabWidgetBase::filterQuads() const
{
    return m_parametersWidget->filterQuads();
}

bool GrabWidgetBase::fastCheck() const
{
    return m_parametersWidget->fastCheck();
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

    connect ( m_parametersWidget, &CameraParametersWidget::parametersChanges, this, &MonocularGrabWidget::updateParameters );

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

void MonocularGrabWidget::updateParameters()
{
    cameraWidget()->setType( m_parametersWidget->templateType() );
    cameraWidget()->setCount( m_parametersWidget->templateCount() );
    cameraWidget()->setTemplateSize( m_parametersWidget->templateSize() );

    cameraWidget()->setAdaptiveThreshold( m_parametersWidget->adaptiveThreshold() );
    cameraWidget()->setNormalizeImage( m_parametersWidget->normalizeImage() );
    cameraWidget()->setFilterQuads( m_parametersWidget->filterQuads() );
    cameraWidget()->setFastCheck( m_parametersWidget->fastCheck() );

    cameraWidget()->setResizeFlag( m_parametersWidget->rescaleFlag() );
    cameraWidget()->setFrameMaximumSize( m_parametersWidget->rescaleSize() );

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

    connect ( m_parametersWidget, &CameraParametersWidget::parametersChanges, this, &StereoGrabWidget::updateParameters );

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

const StampedStereoImage StereoGrabWidget::sourceFrame() const
{
    return cameraWidget()->sourceFrame();
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

void StereoGrabWidget::updateParameters()
{
    cameraWidget()->setType( m_parametersWidget->templateType() );
    cameraWidget()->setCount( m_parametersWidget->templateCount() );
    cameraWidget()->setTemplateSize( m_parametersWidget->templateSize() );

    cameraWidget()->setAdaptiveThreshold( m_parametersWidget->adaptiveThreshold() );
    cameraWidget()->setNormalizeImage( m_parametersWidget->normalizeImage() );
    cameraWidget()->setFilterQuads( m_parametersWidget->filterQuads() );
    cameraWidget()->setFastCheck( m_parametersWidget->fastCheck() );

    cameraWidget()->setResizeFlag( m_parametersWidget->rescaleFlag() );
    cameraWidget()->setFrameMaximumSize( m_parametersWidget->rescaleSize() );

}
