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

    m_smallerBlockSizeLayout = new IntSliderLayout( tr( "Smaller block size" ) );
    m_smallerBlockSizeLayout->setRange( 0, 100 );
    layout->addLayout( m_smallerBlockSizeLayout );

    m_filterLambdaLayout = new IntSliderLayout( tr( "Filter lambda" ) );
    m_filterLambdaLayout->setRange( 0, 50000 );
    layout->addLayout( m_filterLambdaLayout );

    m_lrcThreshLayout = new IntSliderLayout( tr( "LRC Threshold" ) );
    m_lrcThreshLayout->setRange( 0, 100 );
    layout->addLayout( m_lrcThreshLayout );

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
    setSmallerBlockSize( 0 );
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

int BMControlWidget::smallerBlockSize() const
{
    return m_smallerBlockSizeLayout->value();
}

int BMControlWidget::filterLambda() const
{
    return m_filterLambdaLayout->value();
}

int BMControlWidget::lrcThresh() const
{
    return m_lrcThreshLayout->value();
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

void BMControlWidget::setSmallerBlockSize( const int value )
{
    m_smallerBlockSizeLayout->setValue( value );
}

void BMControlWidget::setFilterLambda( const int value )
{
    m_filterLambdaLayout->setValue( value );
}

void BMControlWidget::setLrcThresh( const int value ) const
{
    m_lrcThreshLayout->setValue( value );
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

    m_smallerBlockSizeLayout = new IntSliderLayout( tr( "Smaller block size" ) );
    m_smallerBlockSizeLayout->setRange( 0, 100 );
    layout->addLayout( m_smallerBlockSizeLayout );

    m_filterLambdaLayout = new IntSliderLayout( tr( "Filter lambda" ) );
    m_filterLambdaLayout->setRange( 0, 50000 );
    layout->addLayout( m_filterLambdaLayout );

    m_lrcThreshLayout = new IntSliderLayout( tr( "LRC Threshold" ) );
    m_lrcThreshLayout->setRange( 0, 100 );
    layout->addLayout( m_lrcThreshLayout );

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
    setSmallerBlockSize( 0 );
}

int GMControlWidget::prefilterSize() const
{
    return m_preFilterSizeLayout->value();
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

int GMControlWidget::textureThreshold() const
{
    return m_textureThresholdLayout->value();
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

int GMControlWidget::smallerBlockSize() const
{
    return m_smallerBlockSizeLayout->value();
}

int GMControlWidget::filterLambda() const
{
    return m_filterLambdaLayout->value();
}

int GMControlWidget::lrcThresh() const
{
    return m_lrcThreshLayout->value();
}

void GMControlWidget::setPrefilterSize( const int value )
{
    m_preFilterSizeLayout->setValue( value );
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

void GMControlWidget::setTextureThreshold( const int value )
{
    m_textureThresholdLayout->setValue( value );
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

void GMControlWidget::setSmallerBlockSize( const int value )
{
    m_smallerBlockSizeLayout->setValue( value );
}

void GMControlWidget::setFilterLambda( const int value )
{
    m_filterLambdaLayout->setValue( value );
}

void GMControlWidget::setLrcThresh( const int value ) const
{
    m_lrcThreshLayout->setValue( value );
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

    layout->addStretch();

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
