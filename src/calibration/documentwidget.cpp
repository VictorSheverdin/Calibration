#include "src/common/precompiled.h"

#include "documentwidget.h"

#include "calibrationwidget.h"
#include "reportwidget.h"

// CalibrationDocumentBase
CalibrationDocumentBase::CalibrationDocumentBase( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void CalibrationDocumentBase::initialize()
{
}

MonocularCameraCalibrationDocument *CalibrationDocumentBase::toMonocularCalibrationDocument()
{
    return dynamic_cast< MonocularCameraCalibrationDocument * >( this );
}

const MonocularCameraCalibrationDocument *CalibrationDocumentBase::toMonocularCalibrationDocument() const
{
    return dynamic_cast< const MonocularCameraCalibrationDocument * >( this );
}

bool CalibrationDocumentBase::isMonocularCalibrationDocument() const
{
    return toMonocularCalibrationDocument();
}

StereoCameraCalibrationDocument *CalibrationDocumentBase::toStereoCalibrationDocument()
{
    return dynamic_cast< StereoCameraCalibrationDocument * >( this );
}

const StereoCameraCalibrationDocument *CalibrationDocumentBase::toStereoCalibrationDocument() const
{
    return dynamic_cast< const StereoCameraCalibrationDocument * >( this );
}

bool CalibrationDocumentBase::isStereoCalibrationDocument() const
{
    return toStereoCalibrationDocument();
}

TrippleCalibrationDocument *CalibrationDocumentBase::toTrippleCalibrationDocument()
{
    return dynamic_cast< TrippleCalibrationDocument * >( this );
}

const TrippleCalibrationDocument *CalibrationDocumentBase::toTrippleCalibrationDocument() const
{
    return dynamic_cast< const TrippleCalibrationDocument * >( this );
}

bool CalibrationDocumentBase::isTrippleCalibrationDocument() const
{
    return toTrippleCalibrationDocument();
}

ReportDocumentBase *CalibrationDocumentBase::toReportDocument()
{
    return dynamic_cast< ReportDocumentBase * >( this );
}

const ReportDocumentBase *CalibrationDocumentBase::toReportDocument() const
{
    return dynamic_cast< const ReportDocumentBase * >( this );
}

bool CalibrationDocumentBase::isReportDocument() const
{
    return toReportDocument();
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
    : CalibrationDocumentBase( parent )
{
    initialize();
}

void CameraCalibrationDocumentBase::initialize()
{
}

CalibrationWidgetBase *CameraCalibrationDocumentBase::widget() const
{
    return dynamic_cast< CalibrationWidgetBase * >( m_widget );
}

void CameraCalibrationDocumentBase::importDialog()
{

}

void CameraCalibrationDocumentBase::exportDialog()
{

}

void CameraCalibrationDocumentBase::grabFrame()
{
    auto widget = dynamic_cast< CameraCalibrationWidgetBase * >( m_widget );

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

// MonocularImageCalibrationDocument
MonocularImageCalibrationDocument::MonocularImageCalibrationDocument( QWidget* parent )
    : CalibrationDocumentBase( parent )
{
    initialize();
}

void MonocularImageCalibrationDocument::initialize()
{
    setWidget( new MonocularImageCalibrationWidget( this ) );
}

MonocularImageCalibrationWidget *MonocularImageCalibrationDocument::widget() const
{
    return dynamic_cast< MonocularImageCalibrationWidget * >( m_widget );
}

// StereoImageCalibrationDocument
StereoImageCalibrationDocument::StereoImageCalibrationDocument( QWidget* parent )
    : CalibrationDocumentBase( parent )
{
    initialize();
}

void StereoImageCalibrationDocument::initialize()
{
    setWidget( new StereoImageCalibrationWidget( this ) );
}

StereoImageCalibrationWidget *StereoImageCalibrationDocument::widget() const
{
    return dynamic_cast< StereoImageCalibrationWidget * >( m_widget );
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

// ReportDocumentBase
ReportDocumentBase::ReportDocumentBase( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void ReportDocumentBase::initialize()
{
}

// MonocularReportDocument
MonocularReportDocument::MonocularReportDocument( QWidget* parent )
    : ReportDocumentBase( parent )
{
    initialize();
}

void MonocularReportDocument::initialize()
{
    setWidget( new MonocularReportWidget( this ) );
}

MonocularReportWidget *MonocularReportDocument::widget() const
{
    return dynamic_cast< MonocularReportWidget * >( m_widget );
}

void MonocularReportDocument::report( const MonocularCalibrationData &calibration )
{
    widget()->report( calibration );
}

void MonocularReportDocument::exportYaml() const
{
    widget()->saveYAMLDialog();
}

// StereoReportDocument
StereoReportDocument::StereoReportDocument( QWidget* parent )
    : ReportDocumentBase( parent )
{
    initialize();
}

void StereoReportDocument::initialize()
{
    setWidget( new StereoReportWidget( this ) );
}

StereoReportWidget *StereoReportDocument::widget() const
{
    return dynamic_cast< StereoReportWidget * >( m_widget );
}

void StereoReportDocument::report( const StereoCalibrationData &calibration )
{
    widget()->report( calibration );
}

void StereoReportDocument::exportYaml() const
{
    widget()->saveYAMLDialog();
}
