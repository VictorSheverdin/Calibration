#pragma once

#include <QWidget>
#include <QPointer>

class IntSliderBox;
class DoubleSliderBox;

class DisparityControlWidget : public QWidget
{
    Q_OBJECT

public:
    DisparityControlWidget( QWidget* parent = nullptr );

protected:
    QPointer< IntSliderBox > m_preFilterSizeBox;
    QPointer< IntSliderBox > m_preFilterCapBox;
    QPointer< IntSliderBox > m_sadWindowSizeBox;
    QPointer< IntSliderBox > m_minDisparityBox;
    QPointer< IntSliderBox > m_numDisparitiesBox;
    QPointer< IntSliderBox > m_textureThresholdBox;
    QPointer< IntSliderBox > m_uniquessRatioBox;
    QPointer< IntSliderBox > m_speckleWindowSizeBox;
    QPointer< IntSliderBox > m_speckleRangeBox;

private:
    void initialize();
};
