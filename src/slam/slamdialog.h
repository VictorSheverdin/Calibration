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

    StereoDirWidget *widget() const;

    QString leftDir() const;
    QString rightDir() const;

private:
    void initialize();

};

class CamerasDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit CamerasDialog( QWidget* parent = nullptr );

    StereoIPWidget *widget() const;

    QString leftIp() const;
    QString rightIp() const;

private:
    void initialize();

};

