#include "src/common/precompiled.h"

#include "disparitycontrolwidget.h"

#include "src/common/supportwidgets.h"

DisparityControlWidget::DisparityControlWidget( QWidget* parent )
    : QWidget( nullptr )
{
    initialize();
}

void DisparityControlWidget::initialize()
{
    auto layout = new QVBoxLayout( this );

   m_preFilterSizeBox = new IntSliderBox( tr("Prefilter size" ), this );
   m_preFilterSizeBox->setRange( 5, 255, 2 );
   layout->addWidget( m_preFilterSizeBox );

   m_preFilterCapBox = new IntSliderBox( tr("Prefilter cap" ), this );
   layout->addWidget( m_preFilterCapBox );
   m_preFilterCapBox->setRange( 1, 63 );

   m_sadWindowSizeBox = new IntSliderBox( tr("SAD Window size" ), this );
   m_sadWindowSizeBox->setRange( 5, 255, 2 );
   layout->addWidget( m_sadWindowSizeBox );

   m_minDisparityBox = new IntSliderBox( tr( "Minimum disparity" ), this );
   m_minDisparityBox->setRange( -255, 255 );
   layout->addWidget( m_minDisparityBox );

   m_numDisparitiesBox = new IntSliderBox( tr( "Number of disparities" ), this );
   m_numDisparitiesBox->setRange( 16, 256, 16 );
   layout->addWidget( m_numDisparitiesBox );

   m_textureThresholdBox = new IntSliderBox( tr( "Texture threshold" ), this );
   m_textureThresholdBox->setRange( 0, 1000 );
   layout->addWidget( m_textureThresholdBox );

   m_uniquessRatioBox = new IntSliderBox( tr( "Uniquess ratio" ), this );
   layout->addWidget( m_uniquessRatioBox );

   m_speckleWindowSizeBox = new IntSliderBox( tr( "Speckle window size" ), this );
   layout->addWidget( m_speckleWindowSizeBox );

   m_speckleRangeBox = new IntSliderBox( tr( "Speckle range" ), this );
   layout->addWidget( m_speckleRangeBox );

   m_disp12MaxDiffBox = new IntSliderBox( tr( "Max difference" ), this );
   m_disp12MaxDiffBox->setRange( 0, 1000 );
   layout->addWidget( m_disp12MaxDiffBox );

   m_smallerBlockSizeBox = new IntSliderBox( tr( "Smaller block size" ), this );
   m_smallerBlockSizeBox->setRange( 0, 100 );
   layout->addWidget( m_smallerBlockSizeBox );

   m_filterLambdaBox = new IntSliderBox( tr( "Filter lambda" ), this );
   m_filterLambdaBox->setRange( 0, 50000 );
   layout->addWidget( m_filterLambdaBox );

   m_lrcThreshBox = new IntSliderBox( tr( "LRC Threshold" ), this );
   m_lrcThreshBox->setRange( 0, 100 );
   layout->addWidget( m_lrcThreshBox );

   setSadWindowSize( 5 );
   setPrefilterSize( 5 );
   setPrefilterCap( 1 );
   setMinDisparity( 0 );
   setNumDisparities( 16 );
   setTextureThreshold( 0 );
   setUniquessRatio( 0 );
   setSpeckleWindowSize( 0 );
   setSpeckleRange( 0 );
   setDisp12MaxDiff( 0 );
   setSmallerBlockSize( 0 );

}

int DisparityControlWidget::prefilterSize() const
{
    return m_preFilterSizeBox->value();
}

int DisparityControlWidget::prefilterCap() const
{
    return m_preFilterCapBox->value();
}

int DisparityControlWidget::sadWindowSize() const
{
    return m_sadWindowSizeBox->value();
}

int DisparityControlWidget::minDisparity() const
{
    return m_minDisparityBox->value();
}

int DisparityControlWidget::numDisparities() const
{
    return m_numDisparitiesBox->value();
}

int DisparityControlWidget::textureThreshold() const
{
    return m_textureThresholdBox->value();
}

int DisparityControlWidget::uniquessRatio() const
{
    return m_uniquessRatioBox->value();
}

int DisparityControlWidget::speckleWindowSize() const
{
    return m_speckleWindowSizeBox->value();
}

int DisparityControlWidget::speckleRange() const
{
    return m_speckleRangeBox->value();
}

int DisparityControlWidget::disp12MaxDiff() const
{
    return m_disp12MaxDiffBox->value();
}

int DisparityControlWidget::smallerBlockSize() const
{
    return m_smallerBlockSizeBox->value();
}

int DisparityControlWidget::filterLambda() const
{
    return m_filterLambdaBox->value();
}

int DisparityControlWidget::lrcThresh() const
{
    return m_lrcThreshBox->value();
}

void DisparityControlWidget::setPrefilterSize( const int value )
{
    m_preFilterSizeBox->setValue( value );
}

void DisparityControlWidget::setPrefilterCap( const int value )
{
    m_preFilterCapBox->setValue( value );
}

void DisparityControlWidget::setSadWindowSize( const int value )
{
    m_sadWindowSizeBox->setValue( value );
}

void DisparityControlWidget::setMinDisparity( const int value )
{
    m_minDisparityBox->setValue( value );
}

void DisparityControlWidget::setNumDisparities( const int value )
{
    m_numDisparitiesBox->setValue( value );
}

void DisparityControlWidget::setTextureThreshold( const int value )
{
    m_textureThresholdBox->setValue( value );
}

void DisparityControlWidget::setUniquessRatio( const int value )
{
    m_uniquessRatioBox->setValue( value );
}

void DisparityControlWidget::setSpeckleWindowSize( const int value )
{
    m_speckleWindowSizeBox->setValue( value );
}

void DisparityControlWidget::setSpeckleRange( const int value )
{
    m_speckleRangeBox->setValue( value );
}

void DisparityControlWidget::setDisp12MaxDiff( const int value )
{
    m_disp12MaxDiffBox->setValue( value );
}

void DisparityControlWidget::setSmallerBlockSize( const int value )
{
    m_smallerBlockSizeBox->setValue( value );
}

void DisparityControlWidget::setFilterLambda( const int value )
{
    m_filterLambdaBox->setValue( value );
}

void DisparityControlWidget::setLrcThresh( const int value ) const
{
    m_lrcThreshBox->setValue( value );
}
