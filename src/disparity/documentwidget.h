#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

class CameraDisparityWidget;
class ImageDisparityWidget;

class DisparityDocumentBase : public DocumentBase
{
    Q_OBJECT

public:
    explicit DisparityDocumentBase( QWidget* parent = nullptr );

public slots:
    virtual void loadCalibrationDialog() = 0;

private:
    void initialize();

};


class CameraDisparityDocument : public DisparityDocumentBase
{
    Q_OBJECT

public:
    explicit CameraDisparityDocument( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent = nullptr );

    CameraDisparityWidget *widget() const;

    bool loadCalibrationFile( const QString &fileName );

public slots:
    virtual void loadCalibrationDialog() override;

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};

class ImageDisparityDocument : public DisparityDocumentBase
{
    Q_OBJECT

public:
    explicit ImageDisparityDocument( QWidget* parent = nullptr );

    ImageDisparityWidget *widget() const;

    bool loadCalibrationFile( const QString &fileName );

public slots:
    virtual void loadCalibrationDialog() override;
    void importDialog();

    void clearIcons();

private:
    void initialize();

};
