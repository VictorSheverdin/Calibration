#pragma once

#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include "templateprocessor.h"

class QHBoxLayout;

class TypeComboBox : public QComboBox
{
    Q_OBJECT

public:
    explicit TypeComboBox( QWidget *parent = nullptr );

    TemplateProcessor::Type currentType() const;

private:
    void initialize();

};

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

class SliderBoxBase : public QWidget
{
    Q_OBJECT

public:
    explicit SliderBoxBase( QWidget* parent = nullptr );

protected:
    QPointer< QHBoxLayout > m_layout;
    QPointer< QSlider > m_slider;

private:
    void initialize();
};

class IntSliderBox : public SliderBoxBase
{
    Q_OBJECT

public:
    explicit IntSliderBox( QWidget* parent = nullptr );

protected:
    QPointer< QSpinBox > m_spinBox;

private:
    void initialize();

};

class DoubleSliderBox : public SliderBoxBase
{
    Q_OBJECT

public:
    explicit DoubleSliderBox( QWidget* parent = nullptr );

protected:
    QPointer< QDoubleSpinBox > m_spinBox;

private:
    void initialize();

};
