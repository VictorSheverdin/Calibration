#pragma once

#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include "templateprocessor.h"

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
