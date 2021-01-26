#include "src/common/precompiled.h"

#include "slamdocument.h"

#include "slamwidget.h"

// ImageSlamDocument
ImageSlamDocument::ImageSlamDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, QWidget *parent )
    : DocumentBase( parent )
{
    initialize( leftList, rightList, calibrationFile );
}

void ImageSlamDocument::initialize( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile )
{
    setWidget( new SlamImageWidget( leftList, rightList, calibrationFile, this ) );
}

SlamImageWidget *ImageSlamDocument::widget() const
{
    return dynamic_cast< SlamImageWidget * >( m_widget );
}
