#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

// class CameraSlamWidget;
class SlamImageWidget;

class SlamDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit SlamDocumentBase( QWidget* parent = nullptr );

private:
    void initialize();

};

/*
class CameraSlamDocument : public SlamDocumentBase
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

*/
class ImageSlamDocument : public SlamDocumentBase
{
    Q_OBJECT

public:
    explicit ImageSlamDocument( const QString &calibrationFile, QWidget* parent = nullptr );

    SlamImageWidget *widget() const;

private:
    void initialize( const QString &calibrationFile );

};

