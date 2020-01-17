#include "src/common/precompiled.h"

#include "slamdocument.h"

#include "slamwidget.h"

// DisparityDocumentBase
SlamDocumentBase::SlamDocumentBase( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void SlamDocumentBase::initialize()
{
}
/*
// CalibrationDocumentBase
CameraDisparityDocument::CameraDisparityDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : DisparityDocumentBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void CameraDisparityDocument::initialize( const QString &leftCameraIp, const QString &rightCameraIp )
{
    setWidget( new CameraDisparityWidget( leftCameraIp, rightCameraIp, this ) );
}

CameraDisparityWidget *CameraDisparityDocument::widget() const
{
    return dynamic_cast< CameraDisparityWidget * >( m_widget );
}

void CameraDisparityDocument::loadCalibrationFile( const QString &fileName )
{
    widget()->loadCalibrationFile( fileName );
}

void CameraDisparityDocument::loadCalibrationDialog()
{
    widget()->loadCalibrationDialog();
}
*/
// ImageSlamDocument
ImageSlamDocument::ImageSlamDocument( QWidget* parent )
    : SlamDocumentBase( parent )
{
    initialize();
}

void ImageSlamDocument::initialize()
{
    setWidget( new SlamWidget( this ) );
}

SlamWidget *ImageSlamDocument::widget() const
{
    return dynamic_cast< SlamWidget * >( m_widget );
}


