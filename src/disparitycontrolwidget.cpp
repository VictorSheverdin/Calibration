#include "precompiled.h"

#include "disparitycontrolwidget.h"

#include "supportwidgets.h"

DisparityControlWidget::DisparityControlWidget( QWidget* parent )
    : QWidget( nullptr )
{
    initialize();
}

void DisparityControlWidget::initialize()
{
    auto layout = new QVBoxLayout( this );

   m_preFilterSizeBox = new IntSliderBox( tr("Prefilter size" ), this );
   m_preFilterSizeBox->setMinimum( 5 );
   m_preFilterSizeBox->setMaximum( 255 );
   m_preFilterSizeBox->setStepSize( 2 );
   layout->addWidget( m_preFilterSizeBox );

   m_preFilterCapBox = new IntSliderBox( tr("Prefilter cap" ), this );
   layout->addWidget( m_preFilterCapBox );
   m_preFilterCapBox->setMinimum( 1 );
   m_preFilterCapBox->setMaximum( 63 );

   m_sadWindowSizeBox = new IntSliderBox( tr("SAD Window size" ), this );
   m_sadWindowSizeBox->setMinimum( 5 );
   m_sadWindowSizeBox->setMaximum( 255 );
   m_sadWindowSizeBox->setStepSize( 2 );
   setSadWindowSize(5);
   layout->addWidget( m_sadWindowSizeBox );

   m_minDisparityBox = new IntSliderBox( tr( "Minimum disparity" ), this );
   layout->addWidget( m_minDisparityBox );

   m_numDisparitiesBox = new IntSliderBox( tr( "Number of disparities" ), this );
   m_numDisparitiesBox->setMinimum( 16 );
   m_numDisparitiesBox->setStepSize( 16 );
   m_numDisparitiesBox->setMaximum( 256 );
   layout->addWidget( m_numDisparitiesBox );

   m_textureThresholdBox = new IntSliderBox( tr( "Texture threshold" ), this );
   m_textureThresholdBox->setMinimum( 0 );
   m_textureThresholdBox->setMaximum( 1000 );
   layout->addWidget( m_textureThresholdBox );

   m_uniquessRatioBox = new IntSliderBox( tr( "Uniquess ratio" ), this );
   layout->addWidget( m_uniquessRatioBox );

   m_speckleWindowSizeBox = new IntSliderBox( tr( "Speckle window size" ), this );
   layout->addWidget( m_speckleWindowSizeBox );

   m_speckleRangeBox = new IntSliderBox( tr( "Speckle range" ), this );
   layout->addWidget( m_speckleRangeBox );

   setPrefilterSize(5);
   setPrefilterCap(1);
   setMinDisparity(0);
   setNumDisparities(16);
   setTextureThreshold(0);
   setUniquessRatio(0);
   setSpeckleWindowSize(0);
   setSpeckleRange(0);

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
