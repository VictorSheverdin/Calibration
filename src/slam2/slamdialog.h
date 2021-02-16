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

    QString leftMaskFile() const;
    QString rightMaskFile() const;

protected:
    QPointer< FileLine > _calibrationFileLine;
    QPointer< StereoFilesListWidget > _filesListWidget;
    QPointer< FileLine > _leftMaskLine;
    QPointer< FileLine > _rightMaskLine;

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

    QString leftMaskFile() const;
    QString rightMaskFile() const;

private:
    void initialize();

};
