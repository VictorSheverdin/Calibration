#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

class CameraDisparityWidget;
class ImageDisparityWidget;

class CameraDisparityDocument : public DocumentBase
{
    Q_OBJECT

public:
    explicit CameraDisparityDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

    CameraDisparityWidget *widget() const;

    bool loadCalibrationFile( const QString &fileName );

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};

class ImageDisparityDocument : public DocumentBase
{
    Q_OBJECT

public:
    explicit ImageDisparityDocument( QWidget* parent = nullptr );

    ImageDisparityWidget *widget() const;

    bool loadCalibrationFile( const QString &fileName );

private:
    void initialize();

};
