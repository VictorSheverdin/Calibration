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

class StereoDirWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StereoDirWidget( QWidget *parent = nullptr );
    explicit StereoDirWidget( const QString &leftLabel, const QString &rightLabel, QWidget *parent = nullptr );

    void setLeftLabel( const QString &value );
    void setRightLabel( const QString &value );
    void setLabel( const QString &leftLabel, const QString &rightLabel );
    QString leftLabel() const;
    QString rightLabel() const;

    void setLeftDir( const QString &value );
    void setRightDir( const QString &value );
    void setDir( const QString &leftPath, const QString &rightPath );
    QString leftDir() const;
    QString rightDir() const;

public slots:
    void choiceLeftDirDialog();
    void choiceRightDirDialog();

protected:
    FileLineCollection m_leftCollection;
    FileLineCollection m_rightCollection;

private:
    void initialize();

};

class StereoDirDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit StereoDirDialog( QWidget *parent = nullptr );

    StereoDirWidget *widget() const;

    QString leftDir() const;
    QString rightDir() const;

private:
    void initialize();

};
