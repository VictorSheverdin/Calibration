#include "src/common/precompiled.h"

#include "slamdocument.h"

#include "slamwidget.h"

// ImageSlamDocument
ImageSlamDocument::ImageSlamDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, const QString &leftMaskFile, const QString &rightMaskFile, QWidget* parent )
    : DocumentBase( parent )
{
    initialize( leftList, rightList, calibrationFile, leftMaskFile, rightMaskFile );
}

void ImageSlamDocument::initialize( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, const QString &leftMaskFile, const QString &rightMaskFile )
{
    setWidget( new SlamImageWidget( leftList, rightList, calibrationFile, leftMaskFile, rightMaskFile, this ) );
}

SlamImageWidget *ImageSlamDocument::widget() const
{
    return dynamic_cast< SlamImageWidget * >( m_widget );
}
