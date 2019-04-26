#include "src/common/precompiled.h"

#include "disparitycontrolwidget.h"

#include "src/common/supportwidgets.h"

// IntSliderBox
IntSliderLayout::IntSliderLayout( const QString label, QWidget* parent )
    : QHBoxLayout( parent )
{
    initialize( label );
}

void IntSliderLayout::initialize( const QString label )
{
    m_stepSize = 1;

    m_label = new QLabel( label );
    m_slider = new QSlider( Qt::Horizontal );
    m_numberWidget = new QSpinBox();
    m_numberWidget->setAlignment( Qt::AlignVCenter | Qt::AlignRight );
    m_numberWidget->setFixedWidth( 70 );

    addWidget( m_label );
    addWidget( m_slider );

    addWidget( m_numberWidget );

    connect( m_slider, &QSlider::valueChanged, this, &IntSliderLayout::updateDisplayedValue );

    updateDisplayedValue();

}

int IntSliderLayout::value()
{
    return m_slider->minimum() + ( m_slider->value() - m_slider->minimum() ) * m_stepSize;
}

int IntSliderLayout::stepSize() const
{
    return m_stepSize;
}

int IntSliderLayout::minimum() const
{
    return m_slider->minimum();
}

int IntSliderLayout::maximum() const
{
    return m_slider->minimum() + ( m_slider->maximum() - m_slider->maximum() ) * m_stepSize;
}

void IntSliderLayout::setRange( const int minValue, const int maxValue, const int step )
{
    auto normStep = std::max( 1, step );

    m_stepSize = normStep;

    m_slider->setMinimum( minValue );
    m_slider->setMaximum( minValue + ( maxValue - minValue ) / m_stepSize );

    m_numberWidget->setMinimum( minValue );
    m_numberWidget->setMaximum( maxValue );
    m_numberWidget->setSingleStep( normStep );

}

void IntSliderLayout::setValue( const int value )
{
    auto newValue = m_slider->minimum() + ( value - m_slider->minimum() ) / m_stepSize;

    if ( m_slider->value() != newValue )
        m_slider->setValue( newValue );
}

void IntSliderLayout::updateDisplayedValue()
{
    auto newValue = value();

    if ( m_numberWidget->value() != newValue )
        m_numberWidget->setValue( newValue );
}

// TypeComboBox
TypeComboBox::TypeComboBox(QWidget *parent )
    : QComboBox( parent )
{
    initialize();
}

void TypeComboBox::initialize()
{
    addItem( tr( "Block matching" ), Type::BM );
    addItem( tr( "Global matching" ), Type::GM );
}

TypeComboBox::Type TypeComboBox::currentType() const
{
    return static_cast<Type>( currentData().toInt() );
}

// TypeLayout
TypeLayout::TypeLayout( QWidget* parent )
    : QHBoxLayout( parent )
{
    initialize();
}

void TypeLayout::initialize()
{
    m_label = new QLabel( tr("Algorithm type") );
    addWidget( m_label );

    m_typeComboBox = new TypeComboBox();
    addWidget( m_typeComboBox );

    connect( m_typeComboBox, static_cast< void ( TypeComboBox::* )( int ) >( &TypeComboBox::currentIndexChanged ), this, &TypeLayout::currentIndexChanged );

}

TypeComboBox::Type TypeLayout::value() const
{
    return m_typeComboBox->currentType();
}

// GMTypeComboBox
GMTypeComboBox::GMTypeComboBox( QWidget *parent )
    : QComboBox( parent )
{
    initialize();
}

void GMTypeComboBox::initialize()
{
    addItem( tr( "SGBM" ), Type::SGBM );
    addItem( tr( "HH" ), Type::HH );
    addItem( tr( "SGBM_3WAY" ), Type::SGBM_3WAY );
    addItem( tr( "HH4" ), Type::HH4 );

}

GMTypeComboBox::Type GMTypeComboBox::currentType() const
{
    return static_cast<Type>( currentData().toInt() );
}

// GMTypeLayout
GMTypeLayout::GMTypeLayout( QWidget* parent )
    : QHBoxLayout( parent )
{
    initialize();
}

void GMTypeLayout::initialize()
{
    m_label = new QLabel( tr("Mode") );
    addWidget( m_label );

    m_typeComboBox = new GMTypeComboBox();
    addWidget( m_typeComboBox );

    connect( m_typeComboBox, static_cast< void ( GMTypeComboBox::* )( int ) >( &GMTypeComboBox::currentIndexChanged ), this, &GMTypeLayout::currentIndexChanged );
}

GMTypeComboBox::Type GMTypeLayout::value() const
{
    return m_typeComboBox->currentType();
}

// BMControlWidget
BMControlWidget::BMControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void BMControlWidget::initialize()
{
    auto layout = new QVBoxLayout( this );

    m_preFilterSizeLayout = new IntSliderLayout( tr("Prefilter size" ) );
    m_preFilterSizeLayout->setRange( 5, 255, 2 );
    layout->addLayout( m_preFilterSizeLayout );

    m_preFilterCapLayout = new IntSliderLayout( tr("Prefilter cap" ) );
    layout->addLayout( m_preFilterCapLayout );
    m_preFilterCapLayout->setRange( 1, 63 );

    m_sadWindowSizeLayout = new IntSliderLayout( tr("SAD Window size" ) );
    m_sadWindowSizeLayout->setRange( 5, 255, 2 );
    layout->addLayout( m_sadWindowSizeLayout );

    m_minDisparityLayout = new IntSliderLayout( tr( "Minimum disparity" ) );
    m_minDisparityLayout->setRange( -255, 255 );
    layout->addLayout( m_minDisparityLayout );

    m_numDisparitiesLayout = new IntSliderLayout( tr( "Number of disparities" ) );
    m_numDisparitiesLayout->setRange( 16, 256, 16 );
    layout->addLayout( m_numDisparitiesLayout );

    m_textureThresholdLayout = new IntSliderLayout( tr( "Texture threshold" ) );
    m_textureThresholdLayout->setRange( 0, 1000 );
    layout->addLayout( m_textureThresholdLayout );

    m_uniquessRatioLayout = new IntSliderLayout( tr( "Uniquess ratio" ) );
    layout->addLayout( m_uniquessRatioLayout );

    m_speckleWindowSizeLayout = new IntSliderLayout( tr( "Speckle window size" ) );
    layout->addLayout( m_speckleWindowSizeLayout );

    m_speckleRangeLayout = new IntSliderLayout( tr( "Speckle range" ) );
    layout->addLayout( m_speckleRangeLayout );

    m_disp12MaxDiffLayout = new IntSliderLayout( tr( "Max difference" ) );
    m_disp12MaxDiffLayout->setRange( 0, 1000 );
    layout->addLayout( m_disp12MaxDiffLayout );

    layout->addStretch();

    setSadWindowSize( 12 );
    setPrefilterSize( 100 );
    setPrefilterCap( 50 );
    setMinDisparity( -128 );
    setNumDisparities( 256 );
    setTextureThreshold( 700 );
    setUniquessRatio( 10 );
    setSpeckleWindowSize( 15 );
    setSpeckleRange( 50 );
    setDisp12MaxDiff( 0 );
}

int BMControlWidget::prefilterSize() const
{
    return m_preFilterSizeLayout->value();
}

int BMControlWidget::prefilterCap() const
{
    return m_preFilterCapLayout->value();
}

int BMControlWidget::sadWindowSize() const
{
    return m_sadWindowSizeLayout->value();
}

int BMControlWidget::minDisparity() const
{
    return m_minDisparityLayout->value();
}

int BMControlWidget::numDisparities() const
{
    return m_numDisparitiesLayout->value();
}

int BMControlWidget::textureThreshold() const
{
    return m_textureThresholdLayout->value();
}

int BMControlWidget::uniquessRatio() const
{
    return m_uniquessRatioLayout->value();
}

int BMControlWidget::speckleWindowSize() const
{
    return m_speckleWindowSizeLayout->value();
}

int BMControlWidget::speckleRange() const
{
    return m_speckleRangeLayout->value();
}

int BMControlWidget::disp12MaxDiff() const
{
    return m_disp12MaxDiffLayout->value();
}

void BMControlWidget::setPrefilterSize( const int value )
{
    m_preFilterSizeLayout->setValue( value );
}

void BMControlWidget::setPrefilterCap( const int value )
{
    m_preFilterCapLayout->setValue( value );
}

void BMControlWidget::setSadWindowSize( const int value )
{
    m_sadWindowSizeLayout->setValue( value );
}

void BMControlWidget::setMinDisparity( const int value )
{
    m_minDisparityLayout->setValue( value );
}

void BMControlWidget::setNumDisparities( const int value )
{
    m_numDisparitiesLayout->setValue( value );
}

void BMControlWidget::setTextureThreshold( const int value )
{
    m_textureThresholdLayout->setValue( value );
}

void BMControlWidget::setUniquessRatio( const int value )
{
    m_uniquessRatioLayout->setValue( value );
}

void BMControlWidget::setSpeckleWindowSize( const int value )
{
    m_speckleWindowSizeLayout->setValue( value );
}

void BMControlWidget::setSpeckleRange( const int value )
{
    m_speckleRangeLayout->setValue( value );
}

void BMControlWidget::setDisp12MaxDiff( const int value )
{
    m_disp12MaxDiffLayout->setValue( value );
}

// GMControlWidget
GMControlWidget::GMControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void GMControlWidget::initialize()
{
    auto layout = new QVBoxLayout( this );

    m_modeLayout = new GMTypeLayout();
    layout->addLayout( m_modeLayout );

    m_preFilterCapLayout = new IntSliderLayout( tr("Prefilter cap" ) );
    layout->addLayout( m_preFilterCapLayout );
    m_preFilterCapLayout->setRange( 1, 63 );

    m_sadWindowSizeLayout = new IntSliderLayout( tr("SAD Window size" ) );
    m_sadWindowSizeLayout->setRange( 5, 255, 2 );
    layout->addLayout( m_sadWindowSizeLayout );

    m_minDisparityLayout = new IntSliderLayout( tr( "Minimum disparity" ) );
    m_minDisparityLayout->setRange( -255, 255 );
    layout->addLayout( m_minDisparityLayout );

    m_numDisparitiesLayout = new IntSliderLayout( tr( "Number of disparities" ) );
    m_numDisparitiesLayout->setRange( 16, 256, 16 );
    layout->addLayout( m_numDisparitiesLayout );

    m_uniquessRatioLayout = new IntSliderLayout( tr( "Uniquess ratio" ) );
    layout->addLayout( m_uniquessRatioLayout );

    m_speckleWindowSizeLayout = new IntSliderLayout( tr( "Speckle window size" ) );
    layout->addLayout( m_speckleWindowSizeLayout );

    m_speckleRangeLayout = new IntSliderLayout( tr( "Speckle range" ) );
    layout->addLayout( m_speckleRangeLayout );

    m_disp12MaxDiffLayout = new IntSliderLayout( tr( "Max difference" ) );
    m_disp12MaxDiffLayout->setRange( 0, 1000 );
    layout->addLayout( m_disp12MaxDiffLayout );

    m_p1Layout = new IntSliderLayout( tr( "P1" ) );
    m_p1Layout->setRange( 0, 1000 );
    layout->addLayout( m_p1Layout );

    m_p2Layout = new IntSliderLayout( tr( "P2" ) );
    m_p2Layout->setRange( 0, 1000 );
    layout->addLayout( m_p2Layout );

    layout->addStretch();

    setSadWindowSize( 12 );
    setPrefilterCap( 50 );
    setMinDisparity( -128 );
    setNumDisparities( 256 );
    setUniquessRatio( 10 );
    setSpeckleWindowSize( 15 );
    setSpeckleRange( 50 );
    setDisp12MaxDiff( 0 );
    setP1(0);
    setP2(0);

}

GMTypeComboBox::Type GMControlWidget::mode() const
{
    return m_modeLayout->value();
}

int GMControlWidget::prefilterCap() const
{
    return m_preFilterCapLayout->value();
}

int GMControlWidget::sadWindowSize() const
{
    return m_sadWindowSizeLayout->value();
}

int GMControlWidget::minDisparity() const
{
    return m_minDisparityLayout->value();
}

int GMControlWidget::numDisparities() const
{
    return m_numDisparitiesLayout->value();
}

int GMControlWidget::uniquessRatio() const
{
    return m_uniquessRatioLayout->value();
}

int GMControlWidget::speckleWindowSize() const
{
    return m_speckleWindowSizeLayout->value();
}

int GMControlWidget::speckleRange() const
{
    return m_speckleRangeLayout->value();
}

int GMControlWidget::disp12MaxDiff() const
{
    return m_disp12MaxDiffLayout->value();
}

int GMControlWidget::p1() const
{
    return m_p1Layout->value();
}

int GMControlWidget::p2() const
{
    return m_p2Layout->value();
}

void GMControlWidget::setPrefilterCap( const int value )
{
    m_preFilterCapLayout->setValue( value );
}

void GMControlWidget::setSadWindowSize( const int value )
{
    m_sadWindowSizeLayout->setValue( value );
}

void GMControlWidget::setMinDisparity( const int value )
{
    m_minDisparityLayout->setValue( value );
}

void GMControlWidget::setNumDisparities( const int value )
{
    m_numDisparitiesLayout->setValue( value );
}

void GMControlWidget::setUniquessRatio( const int value )
{
    m_uniquessRatioLayout->setValue( value );
}

void GMControlWidget::setSpeckleWindowSize( const int value )
{
    m_speckleWindowSizeLayout->setValue( value );
}

void GMControlWidget::setSpeckleRange( const int value )
{
    m_speckleRangeLayout->setValue( value );
}

void GMControlWidget::setDisp12MaxDiff( const int value )
{
    m_disp12MaxDiffLayout->setValue( value );
}

void GMControlWidget::setP1(int p1)
{
    m_p1Layout->setValue( p1 );
}

void GMControlWidget::setP2(int p2)
{
    m_p2Layout->setValue( p2 );
}

// FilterControlWidget
FilterControlWidget::FilterControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void FilterControlWidget::initialize()
{

}

// DisparityControlWidget
DisparityControlWidget::DisparityControlWidget( QWidget* parent )
    : QWidget( nullptr )
{
    initialize();
}

void DisparityControlWidget::initialize()
{
    setSizePolicy( QSizePolicy::Minimum, QSizePolicy::Minimum );

    auto layout = new QVBoxLayout( this );

    m_typeLayout = new TypeLayout();
    layout->addLayout( m_typeLayout );

    m_stack = new QStackedWidget( this );

    layout->addWidget( m_stack );

    m_bmControlWidget = new BMControlWidget( this );
    m_gmControlWidget = new GMControlWidget( this );

    m_bmControlIndex = m_stack->addWidget( m_bmControlWidget );
    m_gmControlIndex = m_stack->addWidget( m_gmControlWidget );

    m_filterControlWidget = new FilterControlWidget( this );

    layout->addWidget( m_stack );

    connect( m_typeLayout, &TypeLayout::currentIndexChanged, this, &DisparityControlWidget::updateStackedWidget );

    updateStackedWidget();

}

BMControlWidget *DisparityControlWidget::bmControlWidget() const
{
    return m_bmControlWidget;
}

GMControlWidget *DisparityControlWidget::gmControlWidget() const
{
    return m_gmControlWidget;
}

bool DisparityControlWidget::isBmMethod() const
{
    return m_typeLayout->value() == TypeComboBox::BM;
}

bool DisparityControlWidget::isGmMethod() const
{
    return m_typeLayout->value() == TypeComboBox::GM;
}

void DisparityControlWidget::activateBmWidget() const
{
    m_stack->setCurrentIndex( m_bmControlIndex );
}

void DisparityControlWidget::activateGmWidget() const
{
    m_stack->setCurrentIndex( m_gmControlIndex );
}

void DisparityControlWidget::updateStackedWidget()
{
    if ( m_typeLayout->value() == TypeComboBox::BM )
        activateBmWidget();
    else if (m_typeLayout->value() == TypeComboBox::GM )
        activateGmWidget();

}
