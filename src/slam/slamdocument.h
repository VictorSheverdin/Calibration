#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

class SlamCameraWidget;
class SlamImageWidget;

class SlamDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit SlamDocumentBase( QWidget* parent = nullptr );

private:
    void initialize();

};

class ImageSlamDocument : public SlamDocumentBase
{
    Q_OBJECT

public:
    explicit ImageSlamDocument( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile, QWidget* parent = nullptr );

    SlamImageWidget *widget() const;

private:
    void initialize( const QStringList &leftList, const QStringList &rightList, const QString &calibrationFile );

};


class CameraSlamDocument : public SlamDocumentBase
{
    Q_OBJECT

public:
    explicit CameraSlamDocument( const QString &leftCameraIp, const QString &rightCameraIp, const QString &calibrationFile, QWidget* parent = nullptr );

    SlamCameraWidget *widget() const;

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp , const QString &calibrationFile );

};
