#include "src/common/precompiled.h"

#include "documentwidget.h"

#include "disparitypreviewwidget.h"

// DisparityDocumentBase
DisparityDocumentBase::DisparityDocumentBase( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void DisparityDocumentBase::initialize()
{

}

// CameraDisparityDocument
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

// DiskDisparityDocument
DiskDisparityDocument::DiskDisparityDocument( QWidget* parent )
    : DisparityDocumentBase( parent )
{
    initialize();
}

void DiskDisparityDocument::initialize()
{
}

DiskDisparityWidget *DiskDisparityDocument::widget() const
{
    return dynamic_cast< DiskDisparityWidget * >( m_widget );
}

void DiskDisparityDocument::clearIcons()
{
    widget()->clearIcons();
}

// StereoDisparityDocument
StereoDisparityDocument::StereoDisparityDocument( QWidget* parent )
    : DiskDisparityDocument( parent )
{
    initialize();
}

void StereoDisparityDocument::initialize()
{
    setWidget( new StereoDisparityWidget( this ) );
}

StereoDisparityWidget *StereoDisparityDocument::widget() const
{
    return dynamic_cast< StereoDisparityWidget * >( m_widget );
}

void StereoDisparityDocument::loadCalibrationFile( const QString &fileName )
{
    widget()->loadCalibrationFile( fileName );
}

void StereoDisparityDocument::loadCalibrationDialog()
{
    widget()->loadCalibrationDialog();
}

void StereoDisparityDocument::importDialog()
{
    widget()->importStereoDialog();
}

// FileDisparityDocument
FileDisparityDocument::FileDisparityDocument( QWidget* parent )
    : DiskDisparityDocument( parent )
{
    initialize();
}

void FileDisparityDocument::initialize()
{
    setWidget( new FileDisparityWidget( this ) );
}

FileDisparityWidget *FileDisparityDocument::widget() const
{
    return dynamic_cast< FileDisparityWidget * >( m_widget );
}

void FileDisparityDocument::loadCalibrationFile( const QString &fileName )
{
    widget()->loadCalibrationFile( fileName );
}

void FileDisparityDocument::loadCalibrationDialog()
{
    widget()->loadCalibrationDialog();
}

void FileDisparityDocument::importDialog()
{
    widget()->importDisparityDialog();
}
