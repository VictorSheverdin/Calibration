#include "src/common/precompiled.h"

#include "documentwidget.h"

#include "disparitypreviewwidget.h"

// CalibrationDocumentBase
CameraDisparityDocument::CameraDisparityDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : DocumentBase( parent )
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

bool CameraDisparityDocument::loadCalibrationFile( const QString &fileName )
{
    widget()->loadCalibrationFile( fileName );
}

// ImageDisparityDocument
ImageDisparityDocument::ImageDisparityDocument( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void ImageDisparityDocument::initialize()
{
    setWidget( new ImageDisparityWidget( this ) );
}

ImageDisparityWidget *ImageDisparityDocument::widget() const
{
    return dynamic_cast< ImageDisparityWidget * >( m_widget );
}

bool ImageDisparityDocument::loadCalibrationFile( const QString &fileName )
{
    widget()->loadCalibrationFile( fileName );
}

