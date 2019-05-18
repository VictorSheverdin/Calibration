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

MonocularCalibrationDocument *DocumentBase::toMonocularCalibrationDocument()
{
    return dynamic_cast< MonocularCalibrationDocument * >( this );
}

const MonocularCalibrationDocument *DocumentBase::toMonocularCalibrationDocument() const
{
    return dynamic_cast< const MonocularCalibrationDocument * >( this );
}

bool DocumentBase::isMonocularCalibrationDocument() const
{
    return toMonocularCalibrationDocument();
}

StereoCalibrationDocument *DocumentBase::toStereoCalibrationDocument()
{
    return dynamic_cast< StereoCalibrationDocument * >( this );
}

const StereoCalibrationDocument *DocumentBase::toStereoCalibrationDocument() const
{
    return dynamic_cast< const StereoCalibrationDocument * >( this );
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

void CalibrationDocumentBase::grabFrame()
{
    auto widget = this->widget();

    if ( widget )
        widget->grabFrame();

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

// MonocularCalibrationDocument
MonocularCalibrationDocument::MonocularCalibrationDocument( const QString &cameraIp, QWidget* parent )
    : CalibrationDocumentBase( parent )
{
    initialize( cameraIp );
}

void MonocularCalibrationDocument::initialize( const QString &cameraIp )
{
    setWidget( new MonocularCalibrationWidget( cameraIp, this ) );

}

MonocularCalibrationWidget *MonocularCalibrationDocument::widget() const
{
    return dynamic_cast< MonocularCalibrationWidget * >( m_widget );
}

// StereoCalibrationDocument
StereoCalibrationDocument::StereoCalibrationDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : CalibrationDocumentBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoCalibrationDocument::initialize( const QString &leftCameraIp, const QString &rightCameraIp )
{
    setWidget( new StereoCalibrationWidget( leftCameraIp, rightCameraIp, this ) );
}

StereoCalibrationWidget *StereoCalibrationDocument::widget() const
{
    return dynamic_cast< StereoCalibrationWidget * >( m_widget );
}

// StereoCalibrationDocument
TrippleCalibrationDocument::TrippleCalibrationDocument( QWidget* parent )
    : CalibrationDocumentBase( parent )
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
