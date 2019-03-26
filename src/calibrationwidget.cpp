#include "precompiled.h"

#include "calibrationwidget.h"

#include "taskwidget.h"
#include "iconswidget.h"
#include "camerawidget.h"

CalibrationWidgetBase::CalibrationWidgetBase( QWidget *parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void CalibrationWidgetBase::initialize()
{
}

// MonocularCalibrationWidget
MonocularCalibrationWidget::MonocularCalibrationWidget( const int cameraIndex, QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize( cameraIndex );
}

void MonocularCalibrationWidget::initialize( const int cameraIndex )
{
    m_taskWidget = new MonocularTaskWidget( cameraIndex, this );
    m_iconsWidget = new IconsWidget( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsWidget );
}

void MonocularCalibrationWidget::grabFrame()
{
    m_iconsWidget->insertIcon( new MonocularIcon( m_taskWidget->previewImage(), m_taskWidget->sourceImage(), this ) );
}

// StereoCalibrationWidget
StereoCalibrationWidget::StereoCalibrationWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize( leftCameraIndex, rightCameraIndex );
}

void StereoCalibrationWidget::initialize( const int leftCameraIndex, const int rightCameraIndex )
{
    m_taskWidget = new StereoTaskWidget( leftCameraIndex, rightCameraIndex, this );
    m_iconsWidget = new IconsWidget( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsWidget );
}

void StereoCalibrationWidget::grabFrame()
{
    m_iconsWidget->insertIcon( new StereoIcon(
                                        StereoCameraWidget::createPreview( m_taskWidget->leftDisplayedImage(), m_taskWidget->rightDisplayedImage() ),
                                        m_taskWidget->leftSourceImage(), m_taskWidget->rightSourceImage(), this ) );
}


