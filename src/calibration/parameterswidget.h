#pragma once

#include <QWidget>

#include "templateprocessor.h"

class CountSpinBox;
class SizeSpinBox;
class QCheckBox;
class RescaleSpinBox;

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

class ParametersWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ParametersWidget( QWidget* parent = nullptr );

    TemplateProcessor::Type templateType() const;

    unsigned int xCount() const;
    unsigned int yCount() const;

    const cv::Size templateCount() const;
    double templateSize() const;

signals:
    void parametersChanges();

protected:
    QPointer<QHBoxLayout> m_layout;

    QPointer<TypeComboBox> m_typeCombo;
    QPointer<CountSpinBox> m_xCountSpinBox;
    QPointer<CountSpinBox> m_yCountSpinBox;
    QPointer<SizeSpinBox> m_sizeSpinBox;

private:
    void initialize();

};

class CameraParametersWidget : public ParametersWidget
{
    Q_OBJECT

public:
    explicit CameraParametersWidget( QWidget* parent = nullptr );

    bool adaptiveThreshold() const;
    bool normalizeImage() const;
    bool filterQuads() const;
    bool fastCheck() const;

    bool rescaleFlag() const;

    unsigned int rescaleSize() const;

protected:
    QPointer< QCheckBox > m_adaptiveThresholdCheckBox;
    QPointer< QCheckBox > m_normalizeImageCheckBox;
    QPointer< QCheckBox > m_filterQuadsCheckBox;
    QPointer< QCheckBox > m_fastCheckCheckBox;


    QPointer< QCheckBox > m_rescaleCheckBox;
    QPointer< RescaleSpinBox > m_rescaleSizeSpinBox;

private:
    void initialize();

};

