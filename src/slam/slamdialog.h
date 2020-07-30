#pragma once

#include "src/common/fileslistwidget.h"
#include "src/common/supportwidgets.h"
#include "src/common/ipwidget.h"

class ImagesChoiceWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImagesChoiceWidget( QWidget *parent = nullptr );

    int leftCount() const;
    int rightCount() const;

    QStringList leftFileNames() const;
    QStringList rightFileNames() const;

    QString calibrationFile() const;

protected:
    QPointer< FileLine > m_calibrationFileLine;
    QPointer< StereoFilesListWidget > m_filesListWidget;

private:
    void initialize();

};

class ImagesDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit ImagesDialog( QWidget *parent = nullptr );

    ImagesChoiceWidget *widget() const;

    int leftCount() const;
    int rightCount() const;

    QStringList leftFileNames() const;
    QStringList rightFileNames() const;

    QString calibrationFile() const;

private:
    void initialize();

};

class CamerasChoiceWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CamerasChoiceWidget( QWidget *parent = nullptr );

    QString leftIp() const;
    QString rightIp() const;

    QString calibrationFile() const;

protected:
    QPointer< FileLine > m_calibrationFileLine;
    QPointer< StereoIPWidget > m_camerasIpWidget;

private:
    void initialize();

};

class CamerasDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit CamerasDialog( QWidget* parent = nullptr );

    CamerasChoiceWidget *widget() const;

    QString leftIp() const;
    QString rightIp() const;

    QString calibrationFile() const;

private:
    void initialize();

};

