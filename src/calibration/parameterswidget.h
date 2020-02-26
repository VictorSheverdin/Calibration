#pragma once

#include <QWidget>

class CountSpinBox;
class SizeSpinBox;
class QCheckBox;
class QLabel;
class RescaleSpinBox;

class QHBoxLayout;

class TypeComboBox : public QComboBox
{
    Q_OBJECT

public:
    enum Type { CHECKERBOARD, CIRCLES, ASYM_CIRCLES, ARUCO_MARKERS };

    explicit TypeComboBox( QWidget *parent = nullptr );

    Type currentType() const;

private:
    void initialize();

};

class ParametersWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ParametersWidget( QWidget* parent = nullptr );

    TypeComboBox::Type templateType() const;

    unsigned int xCount() const;
    unsigned int yCount() const;

    const cv::Size templateCount() const;
    double templateSize() const;
    double intervalSize() const;

signals:
    void parametersChanges();

protected slots:
    void updateVisibility();

protected:
    QPointer< QHBoxLayout > m_layout;

    QPointer< QLabel > m_countLabel;
    QPointer< TypeComboBox > m_typeCombo;
    QPointer< CountSpinBox > m_xCountSpinBox;
    QPointer< CountSpinBox > m_yCountSpinBox;
    QPointer< QLabel > m_sizeLabel;
    QPointer< SizeSpinBox > m_sizeSpinBox;
    QPointer< QLabel > m_sizeMeasLabel;
    QPointer< QLabel > m_intervalLabel;
    QPointer< SizeSpinBox > m_intervalSpinBox;
    QPointer< QLabel > m_intervalMeasLabel;

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

