#include "src/common/precompiled.h"

#include "parameterswidget.h"

#include "src/common/supportwidgets.h"
#include "src/common/supportwidgets.h"

// TypeComboBox
TypeComboBox::TypeComboBox(QWidget *parent )
    : QComboBox( parent )
{
    initialize();
}

void TypeComboBox::initialize()
{
    addItem( tr( "Checkerboard" ), TemplateProcessor::CHECKERBOARD );
    addItem( tr( "Circles" ), TemplateProcessor::CIRCLES );
    addItem( tr( "Asymetric circles" ), TemplateProcessor::ASYM_CIRCLES );
    addItem( tr( "Aruco markers" ), TemplateProcessor::ASYM_CIRCLES );
}

TemplateProcessor::Type TypeComboBox::currentType() const
{
    return static_cast<TemplateProcessor::Type>( currentData().toInt() );
}

// ParametersWidget
ParametersWidget::ParametersWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void ParametersWidget::initialize()
{
    QHBoxLayout *layout = new QHBoxLayout( this );

    QHBoxLayout *typeLayout = new QHBoxLayout();

    typeLayout->addWidget( new QLabel( tr( "Template type:" ) ) );

    m_typeCombo = new TypeComboBox(this);
    typeLayout->addWidget( m_typeCombo );

    layout->addLayout( typeLayout );

    QHBoxLayout *countLayout = new QHBoxLayout();

    countLayout->addWidget( new QLabel( tr( "Count:" ) ) );

    m_xCountSpinBox = new CountSpinBox( this );
    m_xCountSpinBox->setValue( 9 );
    m_yCountSpinBox = new CountSpinBox( this );
    m_yCountSpinBox->setValue( 6 );

    countLayout->addWidget( m_xCountSpinBox );
    countLayout->addWidget( m_yCountSpinBox );

    layout->addLayout( countLayout );

    QHBoxLayout *sizeLayout = new QHBoxLayout();

    sizeLayout->addWidget( new QLabel( tr( "Size:" ) ) );

    m_sizeSpinBox = new SizeSpinBox( this );
    m_sizeSpinBox->setValue( 0.05 );
    sizeLayout->addWidget( m_sizeSpinBox );

    layout->addLayout( sizeLayout );

    layout->addStretch();

    m_adaptiveThresholdCheckBox = new QCheckBox( tr( "Adaptive threshold" ), this );
    layout->addWidget( m_adaptiveThresholdCheckBox );
    m_adaptiveThresholdCheckBox->setChecked( true );

    m_normalizeImageCheckBox = new QCheckBox( tr( "Normalize" ), this );
    layout->addWidget( m_normalizeImageCheckBox );
    m_normalizeImageCheckBox->setChecked( true );

    m_filterQuadsCheckBox = new QCheckBox( tr( "Filter quads" ), this );
    layout->addWidget( m_filterQuadsCheckBox );
    m_filterQuadsCheckBox->setChecked( false );

    m_fastCheckCheckBox = new QCheckBox( tr( "Fast check" ), this );
    layout->addWidget( m_fastCheckCheckBox );
    m_fastCheckCheckBox->setChecked( true );

    layout->addStretch();

    m_rescaleCheckBox = new QCheckBox( tr( "Rescale preview" ), this );
    layout->addWidget( m_rescaleCheckBox );
    m_rescaleCheckBox->setChecked( true );

    QHBoxLayout *rescaleLayout = new QHBoxLayout();

    m_rescaleSizeSpinBox = new RescaleSpinBox( this );

    rescaleLayout->addWidget( new QLabel( tr( "Maximum size:" ) ) );
    rescaleLayout->addWidget( m_rescaleSizeSpinBox );

    layout->addLayout( rescaleLayout );

    connect( m_typeCombo, static_cast< void ( TypeComboBox::* )( int ) >( &TypeComboBox::currentIndexChanged ), this, &ParametersWidget::parametersChanges );
    connect( m_xCountSpinBox, static_cast< void ( CountSpinBox::* )( int ) >( &CountSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );
    connect( m_yCountSpinBox, static_cast< void ( CountSpinBox::* )( int ) >( &CountSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );
    connect( m_sizeSpinBox, static_cast< void ( SizeSpinBox::* )( double ) >( &SizeSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );

    connect( m_adaptiveThresholdCheckBox, &QCheckBox::stateChanged , this, &ParametersWidget::parametersChanges );
    connect( m_normalizeImageCheckBox, &QCheckBox::stateChanged , this, &ParametersWidget::parametersChanges );
    connect( m_filterQuadsCheckBox, &QCheckBox::stateChanged , this, &ParametersWidget::parametersChanges );
    connect( m_fastCheckCheckBox, &QCheckBox::stateChanged , this, &ParametersWidget::parametersChanges );

    connect( m_rescaleCheckBox, &QCheckBox::stateChanged , this, &ParametersWidget::parametersChanges );
    connect( m_rescaleSizeSpinBox, static_cast< void ( RescaleSpinBox::* )( int ) >( &RescaleSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );

}

TemplateProcessor::Type ParametersWidget::templateType() const
{
    return m_typeCombo->currentType();
}

unsigned int ParametersWidget::xCount() const
{
    return static_cast< unsigned int >( m_xCountSpinBox->value() );
}

unsigned int ParametersWidget::yCount() const
{
    return static_cast< unsigned int >( m_yCountSpinBox->value() );
}

const cv::Size ParametersWidget::templateCount() const
{
    return cv::Size( xCount(), yCount() );
}

double ParametersWidget::templateSize() const
{
    return m_sizeSpinBox->value();
}

bool ParametersWidget::adaptiveThreshold() const
{
    return m_adaptiveThresholdCheckBox->isChecked();
}

bool ParametersWidget::normalizeImage() const
{
    return m_normalizeImageCheckBox->isChecked();
}

bool ParametersWidget::filterQuads() const
{
    return m_filterQuadsCheckBox->isChecked();
}

bool ParametersWidget::fastCheck() const
{
    m_fastCheckCheckBox->isChecked();
}

bool ParametersWidget::rescaleFlag() const
{
    return m_rescaleCheckBox->isChecked();
}

unsigned int ParametersWidget::rescaleSize() const
{
    return static_cast< unsigned int >( m_rescaleSizeSpinBox->value() );
}
