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
    m_label = new QLabel( label );
    m_slider = new QSlider( Qt::Horizontal );
    m_numberWidget = new QSpinBox();
    m_numberWidget->setAlignment( Qt::AlignVCenter | Qt::AlignRight );
    m_numberWidget->setFixedWidth( 70 );

    addWidget( m_label );
    addWidget( m_slider );

    addWidget( m_numberWidget );

    connect( m_numberWidget, static_cast< void ( QSpinBox::* )( int ) >( &QSpinBox::valueChanged ), this, &IntSliderLayout::updateSliderValue );
    connect( m_slider, &QSlider::valueChanged, this, &IntSliderLayout::updateSpinValue );

    connect( m_numberWidget, static_cast< void ( QSpinBox::* )( int ) >( &QSpinBox::valueChanged ), this, &IntSliderLayout::valueChanged );

    setRange( 0, 100, 1 );

    updateSliderValue();
    updateSpinValue();

}

int IntSliderLayout::value()
{
    return m_numberWidget->value();
}

int IntSliderLayout::stepSize() const
{
    return m_numberWidget->singleStep();
}

int IntSliderLayout::minimum() const
{
    return m_numberWidget->minimum();
}

int IntSliderLayout::maximum() const
{
    return m_numberWidget->maximum();
}

void IntSliderLayout::setRange( const int minValue, const int maxValue, const int step )
{
    auto normStep = std::max( 1, step );

    m_numberWidget->setMinimum( minValue );
    m_numberWidget->setMaximum( maxValue );
    m_numberWidget->setSingleStep( normStep );

    m_slider->setMinimum( 0 );
    m_slider->setMaximum( ( m_numberWidget->maximum() - m_numberWidget->minimum() ) / m_numberWidget->singleStep() );

}

void IntSliderLayout::setValue( const int value )
{
    if ( m_numberWidget->value() != value )
        m_numberWidget->setValue( value );
}

void IntSliderLayout::updateSliderValue()
{
    auto value = ( m_numberWidget->value() - m_numberWidget->minimum() ) / m_numberWidget->singleStep();

    if ( m_slider->value() != value )
        m_slider->setValue( value );

}

void IntSliderLayout::updateSpinValue()
{
    auto value = m_numberWidget->minimum() + m_slider->value() * m_numberWidget->singleStep();

    if ( m_numberWidget->value() != value )
        m_numberWidget->setValue( value );

}

// DoubleSliderLayout
const double DoubleSliderLayout::m_minStepSize = 1e-4;

DoubleSliderLayout::DoubleSliderLayout( const QString label, QWidget* parent )
    : QHBoxLayout( parent )
{
    initialize( label );
}

void DoubleSliderLayout::initialize( const QString label )
{
    m_label = new QLabel( label );
    m_slider = new QSlider( Qt::Horizontal );
    m_numberWidget = new QDoubleSpinBox();
    m_numberWidget->setAlignment( Qt::AlignVCenter | Qt::AlignRight );
    m_numberWidget->setFixedWidth( 70 );

    addWidget( m_label );
    addWidget( m_slider );

    addWidget( m_numberWidget );

    connect( m_numberWidget, static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ), this, &DoubleSliderLayout::updateSliderValue );
    connect( m_slider, &QSlider::valueChanged, this, &DoubleSliderLayout::updateSpinValue );

    connect( m_numberWidget, static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ), this, &DoubleSliderLayout::valueChanged );

    setRange( 0.0, 100.0, 1.0 );

    updateSliderValue();
    updateSpinValue();

}

double DoubleSliderLayout::value()
{
    return m_numberWidget->value();
}

double DoubleSliderLayout::stepSize() const
{
    return m_numberWidget->singleStep();
}

double DoubleSliderLayout::minimum() const
{
    return m_numberWidget->minimum();
}

double DoubleSliderLayout::maximum() const
{
    return m_numberWidget->maximum();
}

void DoubleSliderLayout::setRange( const double minValue, const double maxValue, const double step )
{
    auto normStep = std::max( m_minStepSize, step );

    m_numberWidget->setMinimum( minValue );
    m_numberWidget->setMaximum( maxValue );
    m_numberWidget->setSingleStep( normStep );

    m_slider->setMinimum( 0 );
    m_slider->setMaximum( ( m_numberWidget->maximum() - m_numberWidget->minimum() ) / m_numberWidget->singleStep() );

}

void DoubleSliderLayout::setValue( const double value )
{
    if ( m_numberWidget->value() != value )
        m_numberWidget->setValue( value );
}

void DoubleSliderLayout::updateSliderValue()
{
    auto value = ( m_numberWidget->value() - m_numberWidget->minimum() ) / m_numberWidget->singleStep();

    if ( m_slider->value() != value )
        m_slider->setValue( value );

}

void DoubleSliderLayout::updateSpinValue()
{
    auto value = m_numberWidget->minimum() + m_slider->value() * m_numberWidget->singleStep();

    if ( m_numberWidget->value() != value )
        m_numberWidget->setValue( value );

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
    addItem( tr( "GPU Block matching" ), Type::BM_GPU );
    addItem( tr( "Belief Propagation" ), Type::BP );
    addItem( tr( "Constant Space Belief Propagation" ), Type::CSBP );
    addItem( tr( "ELAS" ), Type::ELAS );
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
    m_disp12MaxDiffLayout->setRange( 0, 10000 );
    layout->addLayout( m_disp12MaxDiffLayout );

    layout->addStretch();

    setSadWindowSize( 7 );
    setPrefilterSize( 15 );
    setPrefilterCap( 7 );
    setMinDisparity( 0 );
    setNumDisparities( 256 );
    setTextureThreshold( 320 );
    setUniquessRatio( 35 );
    setSpeckleWindowSize( 35 );
    setSpeckleRange( 6 );
    setDisp12MaxDiff( 0 );

    connect( m_preFilterSizeLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_preFilterCapLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_sadWindowSizeLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_minDisparityLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_numDisparitiesLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_textureThresholdLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_uniquessRatioLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_speckleWindowSizeLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_speckleRangeLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );
    connect( m_disp12MaxDiffLayout, &IntSliderLayout::valueChanged, this, &BMControlWidget::valueChanged );

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

// BMGPUControlWidget
BMGPUControlWidget::BMGPUControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void BMGPUControlWidget::initialize()
{
    auto layout = new QVBoxLayout( this );

    m_preFilterCapLayout = new IntSliderLayout( tr("Prefilter cap" ) );
    layout->addLayout( m_preFilterCapLayout );
    m_preFilterCapLayout->setRange( 1, 63 );

    m_sadWindowSizeLayout = new IntSliderLayout( tr("SAD Window size" ) );
    m_sadWindowSizeLayout->setRange( 5, 255, 2 );
    layout->addLayout( m_sadWindowSizeLayout );

    m_numDisparitiesLayout = new IntSliderLayout( tr( "Number of disparities" ) );
    m_numDisparitiesLayout->setRange( 16, 256, 16 );
    layout->addLayout( m_numDisparitiesLayout );

    m_textureThresholdLayout = new IntSliderLayout( tr( "Texture threshold" ) );
    m_textureThresholdLayout->setRange( 0, 1000 );
    layout->addLayout( m_textureThresholdLayout );

    layout->addStretch();

    setSadWindowSize( 12 );
    setPrefilterCap( 63 );
    setNumDisparities( 256 );
    setTextureThreshold( 10 );

    connect( m_preFilterCapLayout, &IntSliderLayout::valueChanged, this, &BMGPUControlWidget::valueChanged );
    connect( m_sadWindowSizeLayout, &IntSliderLayout::valueChanged, this, &BMGPUControlWidget::valueChanged );
    connect( m_numDisparitiesLayout, &IntSliderLayout::valueChanged, this, &BMGPUControlWidget::valueChanged );
    connect( m_textureThresholdLayout, &IntSliderLayout::valueChanged, this, &BMGPUControlWidget::valueChanged );

}

int BMGPUControlWidget::prefilterCap() const
{
    return m_preFilterCapLayout->value();
}

int BMGPUControlWidget::sadWindowSize() const
{
    return m_sadWindowSizeLayout->value();
}

int BMGPUControlWidget::numDisparities() const
{
    return m_numDisparitiesLayout->value();
}

int BMGPUControlWidget::textureThreshold() const
{
    return m_textureThresholdLayout->value();
}

void BMGPUControlWidget::setPrefilterCap( const int value )
{
    m_preFilterCapLayout->setValue( value );
}

void BMGPUControlWidget::setSadWindowSize( const int value )
{
    m_sadWindowSizeLayout->setValue( value );
}

void BMGPUControlWidget::setNumDisparities( const int value )
{
    m_numDisparitiesLayout->setValue( value );
}

void BMGPUControlWidget::setTextureThreshold( const int value )
{
    m_textureThresholdLayout->setValue( value );
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
    m_disp12MaxDiffLayout->setRange( 0, 10000 );
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
    setMinDisparity( 0 );
    setNumDisparities( 256 );
    setUniquessRatio( 30 );
    setSpeckleWindowSize( 20 );
    setSpeckleRange( 10 );
    setDisp12MaxDiff( 0 );
    setP1(0);
    setP2(0);

    connect( m_modeLayout, &GMTypeLayout::currentIndexChanged, this, &GMControlWidget::valueChanged );
    connect( m_preFilterCapLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_sadWindowSizeLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_minDisparityLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_numDisparitiesLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_uniquessRatioLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_speckleWindowSizeLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_speckleRangeLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_disp12MaxDiffLayout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_p1Layout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );
    connect( m_p2Layout, &IntSliderLayout::valueChanged, this, &GMControlWidget::valueChanged );

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

// BPControlWidget
BPControlWidget::BPControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void BPControlWidget::initialize()
{
    auto layout = new QVBoxLayout( this );

    m_numDisparitiesLayout = new IntSliderLayout( tr( "Number of disparities" ) );
    m_numDisparitiesLayout->setRange( 16, 256, 16 );
    layout->addLayout( m_numDisparitiesLayout );

    m_numIters = new IntSliderLayout( tr("Number of iterations" ) );
    layout->addLayout( m_numIters );
    m_numIters->setRange( 0, 50 );

    m_numLevels = new IntSliderLayout( tr("Number of levels" ) );
    layout->addLayout( m_numLevels );
    m_numLevels->setRange( 0, 9 );

    m_maxDataTerm = new DoubleSliderLayout( tr("Max data term" ) );
    layout->addLayout( m_maxDataTerm );
    m_maxDataTerm->setRange( 0, 100 );

    m_dataWeight = new DoubleSliderLayout( tr("Data weight" ) );
    layout->addLayout( m_dataWeight );
    m_dataWeight->setRange( 0, 10, 0.01 );

    m_maxDiscTerm = new DoubleSliderLayout( tr("Max disc term" ) );
    layout->addLayout( m_maxDiscTerm );
    m_maxDiscTerm->setRange( 0, 10, 0.1 );

    m_discSingleJump = new DoubleSliderLayout( tr("Disc single jump" ) );
    layout->addLayout( m_discSingleJump );
    m_discSingleJump->setRange( 0, 100 );

    layout->addStretch();

    setNumDisparities( 256 );
    setNumIterations( 5 );
    setNumLevels( 5 );
    setMaxDataTerm( 10.0 );
    setDataWeight( 0.07 );
    setMaxDiscTerm( 1.7 );
    setDiscSingleJump( 1.0 );

    connect( m_numDisparitiesLayout, &IntSliderLayout::valueChanged, this, &BPControlWidget::valueChanged );
    connect( m_numIters, &IntSliderLayout::valueChanged, this, &BPControlWidget::valueChanged );
    connect( m_numLevels, &IntSliderLayout::valueChanged, this, &BPControlWidget::valueChanged );
    connect( m_maxDataTerm, &DoubleSliderLayout::valueChanged, this, &BPControlWidget::valueChanged );
    connect( m_dataWeight, &DoubleSliderLayout::valueChanged, this, &BPControlWidget::valueChanged );
    connect( m_maxDiscTerm, &DoubleSliderLayout::valueChanged, this, &BPControlWidget::valueChanged );
    connect( m_discSingleJump, &DoubleSliderLayout::valueChanged, this, &BPControlWidget::valueChanged );

}

int BPControlWidget::numDisparities() const
{
    return m_numDisparitiesLayout->value();
}

int BPControlWidget::numIterations() const
{
    return m_numIters->value();
}

int BPControlWidget::numLevels() const
{
    return m_numLevels->value();
}

double BPControlWidget::maxDataTerm() const
{
    return m_maxDataTerm->value();
}

double BPControlWidget::dataWeight() const
{
    return m_dataWeight->value();
}

double BPControlWidget::maxDiscTerm() const
{
    return m_maxDiscTerm->value();
}

double BPControlWidget::discSingleJump() const
{
    return m_discSingleJump->value();
}

void BPControlWidget::setNumDisparities( const int value )
{
    m_numDisparitiesLayout->setValue( value );
}

void BPControlWidget::setNumIterations( const int value )
{
    m_numIters->setValue( value );
}

void BPControlWidget::setNumLevels( const int value )
{
    m_numLevels->setValue( value );
}

void BPControlWidget::setMaxDataTerm( const double value )
{
    m_maxDataTerm->setValue( value );
}

void BPControlWidget::setDataWeight( const double value )
{
    m_dataWeight->setValue( value );
}

void BPControlWidget::setMaxDiscTerm( const double value )
{
    m_maxDiscTerm->setValue( value );
}

void BPControlWidget::setDiscSingleJump( const double value )
{
    m_discSingleJump->setValue( value );
}

// CSBPControlWidget
CSBPControlWidget::CSBPControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void CSBPControlWidget::initialize()
{
}

// ElasControlWidget
ElasControlWidget::ElasControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void ElasControlWidget::initialize()
{
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
    : QWidget( parent )
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
    m_bmGpuControlWidget = new BMGPUControlWidget( this );
    m_bpControlWidget = new BPControlWidget( this );
    m_csbpControlWidget = new CSBPControlWidget( this );
    m_elasControlWidget = new ElasControlWidget( this );

    m_bmControlIndex = m_stack->addWidget( m_bmControlWidget );
    m_gmControlIndex = m_stack->addWidget( m_gmControlWidget );
    m_bmGpuControlIndex = m_stack->addWidget( m_bmGpuControlWidget );
    m_bpControlIndex = m_stack->addWidget( m_bpControlWidget );
    m_csbpControlIndex = m_stack->addWidget( m_csbpControlWidget );
    m_elasControlIndex = m_stack->addWidget( m_elasControlWidget );

    m_filterControlWidget = new FilterControlWidget( this );

    layout->addWidget( m_stack );

    connect( m_typeLayout, &TypeLayout::currentIndexChanged, this, &DisparityControlWidget::updateStackedWidget );

    connect( m_typeLayout, &TypeLayout::currentIndexChanged, this, &DisparityControlWidget::valueChanged );

    connect( m_bmControlWidget, &BMControlWidget::valueChanged, this, &DisparityControlWidget::valueChanged );
    connect( m_gmControlWidget, &GMControlWidget::valueChanged, this, &DisparityControlWidget::valueChanged );
    connect( m_bmGpuControlWidget, &BMGPUControlWidget::valueChanged, this, &DisparityControlWidget::valueChanged );
    connect( m_bpControlWidget, &BPControlWidget::valueChanged, this, &DisparityControlWidget::valueChanged );
    connect( m_csbpControlWidget, &CSBPControlWidget::valueChanged, this, &DisparityControlWidget::valueChanged );
    connect( m_elasControlWidget, &ElasControlWidget::valueChanged, this, &DisparityControlWidget::valueChanged );

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

BMGPUControlWidget *DisparityControlWidget::bmGpuControlWidget() const
{
    return m_bmGpuControlWidget;
}

BPControlWidget *DisparityControlWidget::bpControlWidget() const
{
    return m_bpControlWidget;
}

CSBPControlWidget *DisparityControlWidget::csbpControlWidget() const
{
    return m_csbpControlWidget;
}

ElasControlWidget *DisparityControlWidget::elasControlWidget() const
{
    return m_elasControlWidget;
}

bool DisparityControlWidget::isBmMethod() const
{
    return m_typeLayout->value() == TypeComboBox::BM;
}

bool DisparityControlWidget::isGmMethod() const
{
    return m_typeLayout->value() == TypeComboBox::GM;
}

bool DisparityControlWidget::isBmGpuMethod() const
{
    return m_typeLayout->value() == TypeComboBox::BM_GPU;
}

bool DisparityControlWidget::isBpMethod() const
{
    return m_typeLayout->value() == TypeComboBox::BP;
}

bool DisparityControlWidget::isCsbpMethod() const
{
    return m_typeLayout->value() == TypeComboBox::CSBP;
}

bool DisparityControlWidget::isElasMethod() const
{
    return m_typeLayout->value() == TypeComboBox::ELAS;
}

void DisparityControlWidget::activateBmWidget() const
{
    m_stack->setCurrentIndex( m_bmControlIndex );
}

void DisparityControlWidget::activateGmWidget() const
{
    m_stack->setCurrentIndex( m_gmControlIndex );
}

void DisparityControlWidget::activateBmGpuWidget() const
{
    m_stack->setCurrentIndex( m_bmGpuControlIndex );
}

void DisparityControlWidget::activateBpWidget() const
{
    m_stack->setCurrentIndex( m_bpControlIndex );
}

void DisparityControlWidget::activateCsbpWidget() const
{
    m_stack->setCurrentIndex( m_csbpControlIndex );
}

void DisparityControlWidget::activateElasWidget() const
{
    m_stack->setCurrentIndex( m_elasControlIndex );
}

void DisparityControlWidget::activateWidget( const TypeComboBox::Type type ) const
{
    switch( type ) {
    case TypeComboBox::BM :
        activateBmWidget();
        break;
    case TypeComboBox::GM :
        activateGmWidget();
        break;
    case TypeComboBox::BM_GPU :
        activateBmGpuWidget();
        break;
    case TypeComboBox::BP :
        activateBpWidget();
        break;
    case TypeComboBox::CSBP :
        activateCsbpWidget();
        break;
    case TypeComboBox::ELAS :
        activateElasWidget();
        break;

    }
}

void DisparityControlWidget::updateStackedWidget()
{
    activateWidget( m_typeLayout->value() );

}
