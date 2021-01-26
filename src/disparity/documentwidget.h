#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

class CameraDisparityWidget;
class DiskDisparityWidget;
class StereoDisparityWidget;
class FileDisparityWidget;

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

    void loadCalibrationFile( const QString &fileName );

public slots:
    virtual void loadCalibrationDialog() override;

private:
    void initialize( const QString &leftCameraIp, const QString &rightCameraIp );

};

class DiskDisparityDocument : public DisparityDocumentBase
{
    Q_OBJECT

public:
    explicit DiskDisparityDocument( QWidget* parent = nullptr );

    DiskDisparityWidget *widget() const;

public slots:
    void clearIcons();

    virtual void importDialog() = 0;

private:
    void initialize();

};

class StereoDisparityDocument : public DiskDisparityDocument
{
    Q_OBJECT

public:
    explicit StereoDisparityDocument( QWidget* parent = nullptr );

    StereoDisparityWidget *widget() const;

    void loadCalibrationFile( const QString &fileName );

public slots:
    virtual void loadCalibrationDialog() override;
    void importDialog() override;

private:
    void initialize();

};

class FileDisparityDocument : public DiskDisparityDocument
{
    Q_OBJECT

public:
    explicit FileDisparityDocument( QWidget* parent = nullptr );

    FileDisparityWidget *widget() const;

    void loadCalibrationFile( const QString &fileName );

public slots:
    virtual void loadCalibrationDialog() override;
    void importDialog() override;

private:
    void initialize();

};
