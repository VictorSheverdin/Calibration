#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

class SlamImageWidget;

class ImageSlamDocument : public DocumentBase
{
    Q_OBJECT

public:
    explicit ImageSlamDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, const QString &leftMaskFile, const QString &rightMaskFile, QWidget* parent = nullptr );

    SlamImageWidget *widget() const;

private:
    void initialize( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, const QString &leftMaskFile, const QString &rightMaskFile );

};
