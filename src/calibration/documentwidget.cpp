#include "src/common/precompiled.h"

#include "documentwidget.h"

#include "calibrationwidget.h"

// DocumentBase
DocumentBase::DocumentBase( QWidget *widget, QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void DocumentBase::initialize()
{
    m_layout = new QVBoxLayout( this );
}

CalibrationDocumentBase *DocumentBase::toCalibrationDocument()
{
    return dynamic_cast< CalibrationDocumentBase * >( this );
}

const CalibrationDocumentBase *DocumentBase::toCalibrationDocument() const
{
    return dynamic_cast< const CalibrationDocumentBase * >( this );
}

bool DocumentBase::isCalibrationDocument() const
{
    return toCalibrationDocument();
}

MonocularCameraCalibrationDocument *DocumentBase::toMonocularCalibrationDocument()
{
    return dynamic_cast< MonocularCameraCalibrationDocument * >( this );
}

const MonocularCameraCalibrationDocument *DocumentBase::toMonocularCalibrationDocument() const
{
    return dynamic_cast< const MonocularCameraCalibrationDocument * >( this );
}

bool DocumentBase::isMonocularCalibrationDocument() const
{
    return toMonocularCalibrationDocument();
}

StereoCameraCalibrationDocument *DocumentBase::toStereoCalibrationDocument()
{
    return dynamic_cast< StereoCameraCalibrationDocument * >( this );
}

const StereoCameraCalibrationDocument *DocumentBase::toStereoCalibrationDocument() const
{
    return dynamic_cast< const StereoCameraCalibrationDocument * >( this );
}

bool DocumentBase::isStereoCalibrationDocument() const
{
    return toStereoCalibrationDocument();
}

TrippleCalibrationDocument *DocumentBase::toTrippleCalibrationDocument()
{
    return dynamic_cast< TrippleCalibrationDocument * >( this );
}

const TrippleCalibrationDocument *DocumentBase::toTrippleCalibrationDocument() const
{
    return dynamic_cast< const TrippleCalibrationDocument * >( this );
}

bool DocumentBase::isTrippleCalibrationDocument() const
{
    return toTrippleCalibrationDocument();
}

ReportDocument *DocumentBase::toReportDocument()
{
    return dynamic_cast< ReportDocument * >( this );
}

const ReportDocument *DocumentBase::toReportDocument() const
{
    return dynamic_cast< const ReportDocument * >( this );
}

bool DocumentBase::isReportDocument() const
{
    return toReportDocument();
}

void DocumentBase::setWidget( QWidget *widget )
{
    m_widget = widget;

    m_layout->addWidget( widget );
}

QWidget *DocumentBase::widget() const
{
    return m_widget;
}

// CalibrationDocumentBase
CalibrationDocumentBase::CalibrationDocumentBase( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void CalibrationDocumentBase::initialize()
{
}

CalibrationWidgetBase *CalibrationDocumentBase::widget() const
{
    return dynamic_cast< CalibrationWidgetBase * >( m_widget );
}

void CalibrationDocumentBase::importDialog()
{
    auto widget = this->widget();

    if ( widget )
        widget->importDialog();

}

void CalibrationDocumentBase::exportDialog()
{
    auto widget = this->widget();

    if ( widget )
        widget->exportDialog();

}

void CalibrationDocumentBase::calculate()
{
    auto widget = this->widget();

    if ( widget )
        widget->calculate();

}

void CalibrationDocumentBase::clearIcons()
{
    auto widget = this->widget();

    if ( widget )
        widget->clearIcons();

}

// CameraCalibrationDocumentBase
CameraCalibrationDocumentBase::CameraCalibrationDocumentBase( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void CameraCalibrationDocumentBase::initialize()
{
}

CameraCalibrationWidgetBase *CameraCalibrationDocumentBase::widget() const
{
    return dynamic_cast< CameraCalibrationWidgetBase * >( m_widget );
}

void CameraCalibrationDocumentBase::grabFrame()
{
    auto widget = this->widget();

    if ( widget )
        widget->grabFrame();

}

void CameraCalibrationDocumentBase::calculate()
{
    auto widget = this->widget();

    if ( widget )
        widget->calculate();

}

void CameraCalibrationDocumentBase::clearIcons()
{
    auto widget = this->widget();

    if ( widget )
        widget->clearIcons();

}

// MonocularCalibrationDocument
MonocularCalibrationDocument::MonocularCalibrationDocument( QWidget* parent )
    : CalibrationDocumentBase( parent )
{
    initialize();
}

void MonocularCalibrationDocument::initialize()
{
    setWidget( new MonocularCalibrationWidget( this ) );
}

MonocularCalibrationWidget *MonocularCalibrationDocument::widget() const
{
    return dynamic_cast< MonocularCalibrationWidget * >( m_widget );
}

// StereoCalibrationDocument
StereoCalibrationDocument::StereoCalibrationDocument( QWidget* parent )
    : CalibrationDocumentBase( parent )
{
    initialize();
}

void StereoCalibrationDocument::initialize()
{
    setWidget( new StereoCalibrationWidget( this ) );
}

StereoCalibrationWidget *StereoCalibrationDocument::widget() const
{
    return dynamic_cast< StereoCalibrationWidget * >( m_widget );
}

// MonocularCameraCalibrationDocument
MonocularCameraCalibrationDocument::MonocularCameraCalibrationDocument( const QString &cameraIp, QWidget* parent )
    : CameraCalibrationDocumentBase( parent )
{
    initialize( cameraIp );
}

void MonocularCameraCalibrationDocument::initialize( const QString &cameraIp )
{
    setWidget( new MonocularCameraCalibrationWidget( cameraIp, this ) );

}

MonocularCameraCalibrationWidget *MonocularCameraCalibrationDocument::widget() const
{
    return dynamic_cast< MonocularCameraCalibrationWidget * >( m_widget );
}

// StereoCameraCalibrationDocument
StereoCameraCalibrationDocument::StereoCameraCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : CameraCalibrationDocumentBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoCameraCalibrationDocument::initialize( const QString &leftCameraIp, const QString &rightCameraIp )
{
    setWidget( new StereoCameraCalibrationWidget( leftCameraIp, rightCameraIp, this ) );
}

StereoCameraCalibrationWidget *StereoCameraCalibrationDocument::widget() const
{
    return dynamic_cast< StereoCameraCalibrationWidget * >( m_widget );
}

// StereoCameraCalibrationDocument
TrippleCalibrationDocument::TrippleCalibrationDocument( QWidget* parent )
    : CameraCalibrationDocumentBase( parent )
{
    initialize();
}

void TrippleCalibrationDocument::initialize()
{
}

// ReportDocument
ReportDocument::ReportDocument( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void ReportDocument::initialize()
{
}
