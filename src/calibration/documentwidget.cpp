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

// MonocularCalibrationDocument
MonocularCalibrationDocument::MonocularCalibrationDocument( const std::string &cameraIp, QWidget* parent )
    : DocumentBase( parent )
{
    initialize( cameraIp );
}

void MonocularCalibrationDocument::initialize( const std::string &cameraIp )
{
    setWidget( new MonocularCalibrationWidget( cameraIp, this ) );

}

// StereoCalibrationDocument
StereoCalibrationDocument::StereoCalibrationDocument( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent )
    : DocumentBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoCalibrationDocument::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    setWidget( new StereoCalibrationWidget( leftCameraIp, rightCameraIp, this ) );
}

// StereoCalibrationDocument
TrippleCalibrationDocument::TrippleCalibrationDocument( QWidget* parent )
    : DocumentBase( parent )
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
