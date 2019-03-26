#include "precompiled.h"

#include "parameterswidget.h"

#include "supportwidgets.h"

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
    m_yCountSpinBox = new CountSpinBox( this );
    countLayout->addWidget( m_xCountSpinBox );
    countLayout->addWidget( m_yCountSpinBox );

    layout->addLayout( countLayout );

    QHBoxLayout *sizeLayout = new QHBoxLayout();

    sizeLayout->addWidget( new QLabel( tr( "Size:" ) ) );

    m_sizeSpinBox = new SizeSpinBox( this );
    sizeLayout->addWidget( m_sizeSpinBox );

    layout->addLayout( sizeLayout );

    layout->addStretch();

    m_rescaleCheckBox = new QCheckBox( tr( "Rescale preview" ), this );

    layout->addWidget( m_rescaleCheckBox );
    m_rescaleCheckBox->setChecked( false );

    QHBoxLayout *rescaleLayout = new QHBoxLayout();

    m_rescaleSizeSpinBox = new RescaleSpinBox( this );

    rescaleLayout->addWidget( new QLabel( tr( "Maximum size:" ) ) );
    rescaleLayout->addWidget( m_rescaleSizeSpinBox );

    layout->addLayout( rescaleLayout );

    connect( m_typeCombo, static_cast< void ( TypeComboBox::* )( int ) >( &TypeComboBox::currentIndexChanged ), this, &ParametersWidget::parametersChanges );
    connect( m_xCountSpinBox, static_cast< void ( CountSpinBox::* )( int ) >( &CountSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );
    connect( m_yCountSpinBox, static_cast< void ( CountSpinBox::* )( int ) >( &CountSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );
    connect( m_sizeSpinBox, static_cast< void ( SizeSpinBox::* )( double ) >( &SizeSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );
    connect( m_rescaleCheckBox, &QCheckBox::stateChanged , this, &ParametersWidget::parametersChanges );
    connect( m_rescaleSizeSpinBox, static_cast< void ( RescaleSpinBox::* )( int ) >( &RescaleSpinBox::valueChanged ), this, &ParametersWidget::parametersChanges );

}

TemplateProcessor::Type ParametersWidget::type() const
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

const cv::Size ParametersWidget::count() const
{
    return cv::Size( xCount(), yCount() );
}

double ParametersWidget::size() const
{
    return m_sizeSpinBox->value();
}

bool ParametersWidget::rescaleFlag() const
{
    return m_rescaleCheckBox->isChecked();
}

unsigned int ParametersWidget::rescaleSize() const
{
    return static_cast< unsigned int >( m_rescaleSizeSpinBox->value() );
}
