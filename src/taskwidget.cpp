#include "precompiled.h"

#include "taskwidget.h"

#include "parameterswidget.h"
#include "camerawidget.h"

// TaskWidgetBase
TaskWidgetBase::TaskWidgetBase( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void TaskWidgetBase::initialize()
{
    m_layout = new QVBoxLayout( this );

    m_parametersWidget = new ParametersWidget( this );

    m_layout->addWidget( m_parametersWidget );

    connect (m_parametersWidget, &ParametersWidget::parametersChanges, this, &TaskWidgetBase::updateParameters );

}

TemplateProcessor::Type TaskWidgetBase::templateType() const
{
    return m_cameraWidget->templateType();
}

const cv::Size &TaskWidgetBase::templateCount() const
{
    return m_cameraWidget->templateCount();
}

double TaskWidgetBase::templateSize() const
{
    return m_cameraWidget->templateSize();
}

bool TaskWidgetBase::resizeFlag() const
{
    return m_cameraWidget->resizeFlag();
}

unsigned int TaskWidgetBase::frameMaximumFlag() const
{
    return m_cameraWidget->frameMaximumFlag();
}

bool TaskWidgetBase::adaptiveThreshold() const
{
    return m_cameraWidget->adaptiveThreshold();
}

bool TaskWidgetBase::normalizeImage() const
{
    return m_cameraWidget->normalizeImage();
}

bool TaskWidgetBase::filterQuads() const
{
    return m_cameraWidget->filterQuads();
}

bool TaskWidgetBase::fastCheck() const
{
    return m_cameraWidget->fastCheck();
}

void TaskWidgetBase::setCameraDecimation( CameraWidgetBase::DecimationType type )
{
    m_cameraWidget->setDecimation( type );
}

void TaskWidgetBase::updateParameters()
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

// MonocularTaskWidget
MonocularTaskWidget::MonocularTaskWidget( const std::string &cameraIp, QWidget* parent )
    : TaskWidgetBase( parent )
{
    initialize( cameraIp );
}

void MonocularTaskWidget::initialize(const std::string &cameraIp )
{
    m_cameraWidget = new MonocularCameraWidget( cameraIp, this );
    m_layout->addWidget( m_cameraWidget );

    updateParameters();
}

MonocularCameraWidget *MonocularTaskWidget::cameraWidget() const
{
    return dynamic_cast< MonocularCameraWidget * >( m_cameraWidget.data() );
}

const CvImage MonocularTaskWidget::sourceImage() const
{
    return cameraWidget()->sourceImage();
}

const CvImage MonocularTaskWidget::previewImage() const
{
    return cameraWidget()->previewImage();
}

bool MonocularTaskWidget::isTemplateExist() const
{
    return cameraWidget()->isTemplateExist();
}

// StereoTaskWidget
StereoTaskWidget::StereoTaskWidget( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent )
    : TaskWidgetBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoTaskWidget::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    m_cameraWidget = new StereoCameraWidget( leftCameraIp,  rightCameraIp, this );
    m_layout->addWidget( m_cameraWidget );

    updateParameters();
}

StereoCameraWidget *StereoTaskWidget::cameraWidget() const
{
    return dynamic_cast< StereoCameraWidget * >( m_cameraWidget.data() );
}

const CvImage StereoTaskWidget::leftSourceImage() const
{
    return cameraWidget()->leftSourceImage();
}

const CvImage StereoTaskWidget::leftDisplayedImage() const
{
    return cameraWidget()->leftDisplayedImage();
}

const CvImage StereoTaskWidget::rightSourceImage() const
{
    return cameraWidget()->rightSourceImage();
}

const CvImage StereoTaskWidget::rightDisplayedImage() const
{
    return cameraWidget()->rightDisplayedImage();
}

bool StereoTaskWidget::isTemplateExist() const
{
    return cameraWidget()->isTemplateExist();
}

void StereoTaskWidget::setLeftSourceImage( const CvImage image )
{
    return cameraWidget()->setLeftSourceImage( image );
}

void StereoTaskWidget::setLeftDisplayedImage( const CvImage image )
{
    return cameraWidget()->setLeftDisplayedImage( image );
}

void StereoTaskWidget::setRightSourceImage( const CvImage image )
{
    return cameraWidget()->setRightSourceImage( image );
}

void StereoTaskWidget::setRightDisplayedImage( const CvImage image )
{
    return cameraWidget()->setRightDisplayedImage( image );
}
