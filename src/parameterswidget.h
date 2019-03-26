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

    TemplateProcessor::Type type() const;

    unsigned int xCount() const;
    unsigned int yCount() const;

    const cv::Size count() const;

    double size() const;

    bool rescaleFlag() const;

    unsigned int rescaleSize() const;

signals:
    void parametersChanges();

protected:
    QPointer<TypeComboBox> m_typeCombo;
    QPointer<CountSpinBox> m_xCountSpinBox;
    QPointer<CountSpinBox> m_yCountSpinBox;
    QPointer<SizeSpinBox> m_sizeSpinBox;
    QPointer<QCheckBox> m_rescaleCheckBox;
    QPointer<RescaleSpinBox> m_rescaleSizeSpinBox;

private:
    void initialize();

};

