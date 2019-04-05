#pragma once

#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include "templateprocessor.h"

class QHBoxLayout;
class QLabel;

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

class IntSliderBox : public QWidget
{
    Q_OBJECT

public:
    explicit IntSliderBox( const QString label, QWidget* parent = nullptr );

    int value();

    int stepSize() const;
    int minimum() const;
    int maximum() const;

public slots:
    void setRange( const int minValue, const int maxValue, const int step = 1 );

    void setValue( const int value );

protected:
    QPointer< QHBoxLayout > m_layout;
    QPointer< QLabel > m_label;
    QPointer< QSlider > m_slider;
    QPointer< QLineEdit > m_displayWidget;

    int m_stepSize;

    void updateDisplayedValue();

private:
    void initialize( const QString label );
};

