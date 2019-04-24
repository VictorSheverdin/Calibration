#pragma once

#include <QWidget>
#include <QPointer>

class IntSliderBox;

class DisparityControlWidget : public QWidget
{
    Q_OBJECT

public:
    DisparityControlWidget( QWidget* parent = nullptr );

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
    int smallerBlockSize() const;
    int filterLambda() const;
    int lrcThresh() const;

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
    void setSmallerBlockSize( const int value );
    void setFilterLambda( const int value );
    void setLrcThresh( const int value ) const;

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
    QPointer< IntSliderBox > m_disp12MaxDiffBox;
    QPointer< IntSliderBox > m_smallerBlockSizeBox;
    QPointer< IntSliderBox > m_filterLambdaBox;
    QPointer< IntSliderBox > m_lrcThreshBox;

private:
    void initialize();

};
