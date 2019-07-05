#pragma once

#include "supportwidgets.h"
#include <QSplitter>

class FilesListWidget;
class QVBoxLayout;
class QToolBar;

class MonocularFilesListWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MonocularFilesListWidget( QWidget* parent = nullptr );

    int count() const;

    QStringList fileNames() const;

public slots:
    void addDialog();
    void removeDialog();

protected:
    QPointer< QVBoxLayout > m_layout;

    QPointer< QToolBar > m_toolBar;

    QPointer< QAction > m_addAction;
    QPointer< QAction > m_removeAction;

    QPointer< FilesListWidget > m_list;

private:
    void initialize();

};

class StereoFilesListWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit StereoFilesListWidget( QWidget* parent = nullptr );

    int leftCount() const;
    int rightCount() const;

    QStringList leftFileNames() const;
    QStringList rightFileNames() const;

protected:
    QPointer< MonocularFilesListWidget > m_leftList;
    QPointer< MonocularFilesListWidget > m_rightList;

private:
    void initialize();

};

class StereoFilesListDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit StereoFilesListDialog( QWidget *parent = nullptr );

    StereoFilesListWidget *widget() const;

    int leftCount() const;
    int rightCount() const;

    QStringList leftFileNames() const;
    QStringList rightFileNames() const;

protected slots:
    void onAccept();

private:
    void initialize();

};
