#pragma once

#include "src/common/fileslistwidget.h"
#include "src/common/supportwidgets.h"

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

    ImagesChoiceWidget *widget() const;

    int leftCount() const;
    int rightCount() const;

    QStringList leftFileNames() const;
    QStringList rightFileNames() const;

protected slots:
    void onAccept();

private:
    void initialize();

};

class CamerasDialog
{
};
