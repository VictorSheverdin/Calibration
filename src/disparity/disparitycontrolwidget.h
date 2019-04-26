#pragma once

#include <QWidget>
#include <QComboBox>
#include <QBoxLayout>
#include <QPointer>

#include <opencv/cv.hpp>

class QHBoxLayout;
class QLabel;
class QSlider;
class QSpinBox;
class QStackedWidget;

class IntSliderLayout : public QHBoxLayout
{
    Q_OBJECT

public:
    explicit IntSliderLayout( const QString label, QWidget* parent = nullptr );

    int value();

    int stepSize() const;
    int minimum() const;
    int maximum() const;

public slots:
    void setRange( const int minValue, const int maxValue, const int step = 1 );

    void setValue( const int value );

protected:
    QPointer< QLabel > m_label;
    QPointer< QSlider > m_slider;
    QPointer< QSpinBox > m_numberWidget;

    int m_stepSize;

    void updateDisplayedValue();

private:
    void initialize( const QString label );

};

class TypeComboBox : public QComboBox
{
    Q_OBJECT

public:
    enum Type { BM, GM };

    explicit TypeComboBox( QWidget *parent = nullptr );

    Type currentType() const;

private:
    void initialize();

};

class TypeLayout : public QHBoxLayout
{
    Q_OBJECT

public:
    explicit TypeLayout( QWidget* parent = nullptr );

    TypeComboBox::Type value() const;

signals:
    void currentIndexChanged(int index);

protected:
    QPointer< QLabel > m_label;
    QPointer< TypeComboBox > m_typeComboBox;

private:
    void initialize();

};

class GMTypeComboBox : public QComboBox
{
    Q_OBJECT

public:
    enum Type {         SGBM = cv::StereoSGBM::MODE_SGBM,
                        HH   = cv::StereoSGBM::MODE_HH,
                        SGBM_3WAY = cv::StereoSGBM::MODE_SGBM_3WAY,
                        HH4  = cv::StereoSGBM::MODE_HH4
              };

    explicit GMTypeComboBox( QWidget *parent = nullptr );

    Type currentType() const;

private:
    void initialize();

};

class GMTypeLayout : public QHBoxLayout
{
    Q_OBJECT

public:
    explicit GMTypeLayout( QWidget* parent = nullptr );

    GMTypeComboBox::Type value() const;

signals:
    void currentIndexChanged(int index);

protected:
    QPointer< QLabel > m_label;
    QPointer< GMTypeComboBox > m_typeComboBox;

private:
    void initialize();

};

class BMControlWidget : public QWidget
{
    Q_OBJECT

public:
    BMControlWidget( QWidget* parent = nullptr );

    int prefilterSize() const;
    int prefilterCap() const;
    int sadWindowSize() const;
    int minDisparity() const;
    int numDisparities() const;
    int textureThreshold() const;
    int uniquessRatio() const;
    int speckleWindowSize() const;
    int speckleRange() const;
    int disp12MaxDiff() const;

public slots:
    void setPrefilterSize( const int value );
    void setPrefilterCap( const int value );
    void setSadWindowSize( const int value );
    void setMinDisparity( const int value );
    void setNumDisparities( const int value );
    void setTextureThreshold( const int value );
    void setUniquessRatio( const int value );
    void setSpeckleWindowSize( const int value );
    void setSpeckleRange( const int value );
    void setDisp12MaxDiff( const int value );

protected:
    QPointer< IntSliderLayout > m_preFilterSizeLayout;
    QPointer< IntSliderLayout > m_preFilterCapLayout;
    QPointer< IntSliderLayout > m_sadWindowSizeLayout;
    QPointer< IntSliderLayout > m_minDisparityLayout;
    QPointer< IntSliderLayout > m_numDisparitiesLayout;
    QPointer< IntSliderLayout > m_textureThresholdLayout;
    QPointer< IntSliderLayout > m_uniquessRatioLayout;
    QPointer< IntSliderLayout > m_speckleWindowSizeLayout;
    QPointer< IntSliderLayout > m_speckleRangeLayout;
    QPointer< IntSliderLayout > m_disp12MaxDiffLayout;

private:
    void initialize();
};

class GMControlWidget : public QWidget
{
    Q_OBJECT

public:
    GMControlWidget( QWidget* parent = nullptr );

    GMTypeComboBox::Type mode() const;
    int prefilterCap() const;
    int sadWindowSize() const;
    int minDisparity() const;
    int numDisparities() const;
    int uniquessRatio() const;
    int speckleWindowSize() const;
    int speckleRange() const;
    int disp12MaxDiff() const;
    int p1() const;
    int p2() const;

public slots:
    void setPrefilterCap( const int value );
    void setSadWindowSize( const int value );
    void setMinDisparity( const int value );
    void setNumDisparities( const int value );
    void setUniquessRatio( const int value );
    void setSpeckleWindowSize( const int value );
    void setSpeckleRange( const int value );
    void setDisp12MaxDiff( const int value );
    void setP1(int p1);
    void setP2(int p2);

protected:
    QPointer< GMTypeLayout > m_modeLayout;
    QPointer< IntSliderLayout > m_preFilterCapLayout;
    QPointer< IntSliderLayout > m_sadWindowSizeLayout;
    QPointer< IntSliderLayout > m_minDisparityLayout;
    QPointer< IntSliderLayout > m_numDisparitiesLayout;
    QPointer< IntSliderLayout > m_uniquessRatioLayout;
    QPointer< IntSliderLayout > m_speckleWindowSizeLayout;
    QPointer< IntSliderLayout > m_speckleRangeLayout;
    QPointer< IntSliderLayout > m_disp12MaxDiffLayout;
    QPointer< IntSliderLayout > m_p1Layout;
    QPointer< IntSliderLayout > m_p2Layout;

private:
    void initialize();
};

class FilterControlWidget : public QWidget
{
    Q_OBJECT

public:
    FilterControlWidget( QWidget* parent = nullptr );

private:
    void initialize();
};

class DisparityControlWidget : public QWidget
{
    Q_OBJECT

public:
    DisparityControlWidget( QWidget* parent = nullptr );

    BMControlWidget *bmControlWidget() const;
    GMControlWidget *gmControlWidget() const;

    bool isBmMethod() const;
    bool isGmMethod() const;

public slots:
    void activateBmWidget() const;
    void activateGmWidget() const;

protected slots:
    void updateStackedWidget();

protected:
    QPointer< TypeLayout > m_typeLayout;

    QPointer< QStackedWidget > m_stack;

    QPointer< BMControlWidget > m_bmControlWidget;
    QPointer< GMControlWidget > m_gmControlWidget;
    QPointer< FilterControlWidget > m_filterControlWidget;

    int m_bmControlIndex;
    int m_gmControlIndex;

private:
    void initialize();

};
