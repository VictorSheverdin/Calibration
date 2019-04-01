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

void TaskWidgetBase::updateParameters()
{
    m_cameraWidget->setType( m_parametersWidget->type() );
    m_cameraWidget->setCount( m_parametersWidget->count() );
    m_cameraWidget->setSize( m_parametersWidget->size() );

    m_cameraWidget->setAdaptiveThreshold( m_parametersWidget->adaptiveThreshold() );
    m_cameraWidget->setNormalizeImage( m_parametersWidget->normalizeImage() );
    m_cameraWidget->setFilterQuads( m_parametersWidget->filterQuads() );
    m_cameraWidget->setFastCheck( m_parametersWidget->fastCheck() );

    m_cameraWidget->setResizeFlag( m_parametersWidget->rescaleFlag() );
    m_cameraWidget->setFrameMaximumSize( m_parametersWidget->rescaleSize() );

}

// MonocularTaskWidget
MonocularTaskWidget::MonocularTaskWidget( const int cameraIndex, QWidget* parent )
    : TaskWidgetBase( parent )
{
    initialize( cameraIndex );
}

void MonocularTaskWidget::initialize( const int cameraIndex )
{
    m_cameraWidget = new MonocularCameraWidget( cameraIndex, this );
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

// StereoTaskWidget
StereoTaskWidget::StereoTaskWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent )
    : TaskWidgetBase( parent )
{
    initialize( leftCameraIndex, rightCameraIndex );
}

void StereoTaskWidget::initialize( const int leftCameraIndex, const int rightCameraIndex )
{
    m_cameraWidget = new StereoCameraWidget( leftCameraIndex,  rightCameraIndex, this );
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
