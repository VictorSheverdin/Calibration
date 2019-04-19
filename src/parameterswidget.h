#pragma once

#include <QWidget>

#include "templateprocessor.h"

class TypeComboBox;
class CountSpinBox;
class SizeSpinBox;
class QCheckBox;
class RescaleSpinBox;

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

    bool adaptiveThreshold() const;
    bool normalizeImage() const;
    bool filterQuads() const;
    bool fastCheck() const;

    bool rescaleFlag() const;

    unsigned int rescaleSize() const;

signals:
    void parametersChanges();

protected:
    QPointer<TypeComboBox> m_typeCombo;
    QPointer<CountSpinBox> m_xCountSpinBox;
    QPointer<CountSpinBox> m_yCountSpinBox;
    QPointer<SizeSpinBox> m_sizeSpinBox;

    QPointer<QCheckBox> m_adaptiveThresholdCheckBox;
    QPointer<QCheckBox> m_normalizeImageCheckBox;
    QPointer<QCheckBox> m_filterQuadsCheckBox;
    QPointer<QCheckBox> m_fastCheckCheckBox;


    QPointer<QCheckBox> m_rescaleCheckBox;
    QPointer<RescaleSpinBox> m_rescaleSizeSpinBox;

private:
    void initialize();

};

