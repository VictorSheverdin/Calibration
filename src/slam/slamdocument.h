#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

/*
class CameraSlamWidget;
class ImageSlamWidget;

class SlamDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit SlamDocumentBase( QWidget* parent = nullptr );

public slots:
    virtual void loadCalibrationDialog() = 0;

private:
    void initialize();

};


class CameraDisparityDocument : public SlamDocumentBase
{
    Q_OBJECT

public:
    explicit CameraSlamDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

    CameraSlamWidget *widget() const;

    void loadCalibrationFile( const QString &fileName );

public slots:
    virtual void loadCalibrationDialog() override;

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};


class ImageSlamDocument : public SlamDocumentBase
{
    Q_OBJECT

public:
    explicit ImageSlamDocument( QWidget* parent = nullptr );

    ImageSlamWidget *widget() const;

    void loadCalibrationFile( const QString &fileName );

public slots:
    virtual void loadCalibrationDialog() override;

    void importDialog();

private:
    void initialize();

};
*/
