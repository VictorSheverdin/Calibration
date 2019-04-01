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

   m_preFilterSizeBox = new IntSliderBox( this );
   layout->addWidget( m_preFilterSizeBox );
   m_preFilterCapBox = new IntSliderBox( this );
   layout->addWidget( m_preFilterCapBox );
   m_sadWindowSizeBox = new IntSliderBox( this );
   layout->addWidget( m_sadWindowSizeBox );
   m_minDisparityBox = new IntSliderBox( this );
   layout->addWidget( m_minDisparityBox );
   m_numDisparitiesBox = new IntSliderBox( this );
   layout->addWidget( m_numDisparitiesBox );
   m_textureThresholdBox = new IntSliderBox( this );
   layout->addWidget( m_textureThresholdBox );
   m_uniquessRatioBox = new IntSliderBox( this );
   layout->addWidget( m_uniquessRatioBox );
   m_speckleWindowSizeBox = new IntSliderBox( this );
   layout->addWidget( m_speckleWindowSizeBox );
   m_speckleRangeBox = new IntSliderBox( this );
   layout->addWidget( m_speckleRangeBox );




}
