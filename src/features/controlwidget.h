#pragma once

#include <QComboBox>
#include <QPointer>
#include <QHBoxLayout>

class QLabel;
class QStackedWidget;
class QDoubleSpinBox;

class DetectorComboBox : public QComboBox
{
    Q_OBJECT

public:
    enum Type { ShiTomasi, FAST, SIFT, SURF, ORB, KAZE, AKAZE, SUPERGLUE  };

    explicit DetectorComboBox( QWidget *parent = nullptr );

    Type currentType() const;
    void setCurrentType( const Type value );

private:
    void initialize();

};

class DescriptorComboBox : public QComboBox
{
    Q_OBJECT

public:
    enum Type { LK, DAISY, FREAK, SIFT, SURF, ORB, KAZE, AKAZE, SUPERGLUE };

    explicit DescriptorComboBox( QWidget *parent = nullptr );

    Type currentType() const;
    void setCurrentType( const Type value );

private:
    void initialize();

};

class DetectorLayout : public QHBoxLayout
{
    Q_OBJECT

public:
    explicit DetectorLayout( QWidget* parent = nullptr );

    DetectorComboBox::Type value() const;
    void setValue( const DetectorComboBox::Type value );

signals:
    void currentIndexChanged( int index );

protected:
    QPointer< QLabel > m_label;
    QPointer< DetectorComboBox > m_typeComboBox;

private:
    void initialize();

};

class DescriptorLayout : public QHBoxLayout
{
    Q_OBJECT

public:
    explicit DescriptorLayout( QWidget* parent = nullptr );

    DescriptorComboBox::Type value() const;
    void setValue( const DescriptorComboBox::Type value );

signals:
    void currentIndexChanged( int index );

protected:
    QPointer< QLabel > m_label;
    QPointer< DescriptorComboBox > m_typeComboBox;

private:
    void initialize();

};

class FeaturesControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FeaturesControlWidget( QWidget* parent = nullptr );

    double scaleFactor() const;

    bool isShiTomasiDetector() const;
    bool isFastDetector() const;
    bool isSiftDetector() const;
    bool isSurfDetector() const;
    bool isOrbDetector() const;
    bool isKazeDetector() const;
    bool isAkazeDetector() const;
    bool isSuperGlueDetector() const;

    bool isLKFlow() const;
    bool isDaisyDescriptor() const;
    bool isFreakDescriptor() const;
    bool isSiftDescriptor() const;
    bool isSurfDescriptor() const;
    bool isKazeDescriptor() const;
    bool isAkazeDescriptor() const;
    bool isSuperGlueDescriptor() const;

signals:
    void valueChanged();

protected slots:
    void checkValues();
    void updateStackedWidget();

protected:
    QPointer< QDoubleSpinBox > m_scaleFactorSpinBox;
    QPointer< DetectorLayout > m_detectorLayout;
    QPointer< DescriptorLayout > m_descriptorLayout;
    QPointer< QStackedWidget > m_detectorStack;
    QPointer< QStackedWidget > m_descriptorStack;

private:
    void initialize();

};

