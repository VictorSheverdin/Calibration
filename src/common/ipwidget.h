#pragma once

#include <QWidget>

#include "supportwidgets.h"

class QGridLayout;
class QLineEdit;

class IPWidgetBase : public QWidget
{
    Q_OBJECT

public:
    explicit IPWidgetBase( QWidget* parent = nullptr );

protected:
    QPointer< QGridLayout > m_layout;

private:
    void initialize();

};

class CameraIPWidget : public IPWidgetBase
{
    Q_OBJECT

public:
    explicit CameraIPWidget( QWidget* parent = nullptr );

    QString ip() const;

protected:
    QPointer< QLineEdit > m_ipLine;

private:
    void initialize();

};

class StereoIPWidget : public IPWidgetBase
{
    Q_OBJECT

public:
    explicit StereoIPWidget( QWidget* parent = nullptr );

    QString leftIp() const;
    QString rightIp() const;

protected:
    QPointer< QLineEdit > m_leftIpLine;
    QPointer< QLineEdit > m_rightIpLine;

private:
    void initialize();

};

class CameraIPDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit CameraIPDialog( QWidget* parent = nullptr );

    CameraIPWidget *widget() const;

    QString ip() const;

private:
    void initialize();

};

class StereoIPDialog : public DialogBase
{
    Q_OBJECT

public:
    explicit StereoIPDialog( QWidget* parent = nullptr );

    StereoIPWidget *widget() const;

    QString leftIp() const;
    QString rightIp() const;

private:
    void initialize();

};
