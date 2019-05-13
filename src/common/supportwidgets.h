#pragma once

#include <QComboBox>
#include <QSpinBox>
#include <QPointer>

#include <QDialog>

class QHBoxLayout;
class QLabel;

class CountSpinBox : public QSpinBox
{
    Q_OBJECT

public:
    explicit CountSpinBox( QWidget *parent = nullptr );

private:
    void initialize();

};

class SizeSpinBox : public QDoubleSpinBox
{
    Q_OBJECT

public:
    explicit SizeSpinBox( QWidget *parent = nullptr );

private:
    void initialize();

};

class RescaleSpinBox : public QSpinBox
{
    Q_OBJECT

public:
    explicit RescaleSpinBox( QWidget *parent = nullptr );

private:
    void initialize();

};

class DialogBase : public QDialog
{
    Q_OBJECT

public:
    explicit DialogBase( QWidget *parent = nullptr );

private:
    void initialize();

};
