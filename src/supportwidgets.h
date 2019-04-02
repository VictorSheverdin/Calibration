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

class SliderBoxBase : public QWidget
{
    Q_OBJECT

public:
    explicit SliderBoxBase( const QString label, QWidget* parent = nullptr );

protected:
    QPointer< QHBoxLayout > m_layout;
    QPointer< QLabel > m_label;
    QPointer< QSlider > m_slider;

private:
    void initialize( const QString label );
};

class IntSliderBox : public SliderBoxBase
{
    Q_OBJECT

public:
    explicit IntSliderBox( const QString label, QWidget* parent = nullptr );

    int value();

public slots:
    void setMinimum( const int value );
    void setMaximum( const int value );
    void setStepSize( const int value );

    void setValue( const int value );

protected:
    QPointer< QSpinBox > m_spinBox;

private:
    void initialize();

};

class DoubleSliderBox : public SliderBoxBase
{
    Q_OBJECT

public:
    explicit DoubleSliderBox( const QString label, QWidget* parent = nullptr );

    double value();

public slots:
    void setMinimum( const double value );
    void setMaximum( const double value );
    void setStepSize( const double value );

    void setValue( const double value );

protected:
    QPointer< QDoubleSpinBox > m_spinBox;

private:
    void initialize();

};
