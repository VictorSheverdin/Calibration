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
    m_layout = new QHBoxLayout( this );

    QHBoxLayout *typeLayout = new QHBoxLayout();

    typeLayout->addWidget( new QLabel( tr( "Template type:" ) ) );

    m_typeCombo = new TypeComboBox(this);
    typeLayout->addWidget( m_typeCombo );

    m_layout->addLayout( typeLayout );

    QHBoxLayout *countLayout = new QHBoxLayout();

    countLayout->addWidget( new QLabel( tr( "Count:" ) ) );

    m_xCountSpinBox = new CountSpinBox( this );
    m_xCountSpinBox->setValue( 9 );
    m_yCountSpinBox = new CountSpinBox( this );
    m_yCountSpinBox->setValue( 6 );

    countLayout->addWidget( m_xCountSpinBox );
    countLayout->addWidget( m_yCountSpinBox );

    m_layout->addLayout( countLayout );

    QHBoxLayout *sizeLayout = new QHBoxLayout();

    sizeLayout->addWidget( new QLabel( tr( "Size:" ) ) );

    m_sizeSpinBox = new SizeSpinBox( this );
    m_sizeSpinBox->setValue( 0.05 );
    sizeLayout->addWidget( m_sizeSpinBox );

    m_layout->addLayout( sizeLayout );

    m_layout->addStretch();

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

// CameraParametersWidget
CameraParametersWidget::CameraParametersWidget( QWidget* parent )
    : ParametersWidget( parent )
{
    initialize();
}

void CameraParametersWidget::initialize()
{
    m_adaptiveThresholdCheckBox = new QCheckBox( tr( "Adaptive threshold" ), this );
    m_layout->addWidget( m_adaptiveThresholdCheckBox );
    m_adaptiveThresholdCheckBox->setChecked( true );

    m_normalizeImageCheckBox = new QCheckBox( tr( "Normalize" ), this );
    m_layout->addWidget( m_normalizeImageCheckBox );
    m_normalizeImageCheckBox->setChecked( true );

    m_filterQuadsCheckBox = new QCheckBox( tr( "Filter quads" ), this );
    m_layout->addWidget( m_filterQuadsCheckBox );
    m_filterQuadsCheckBox->setChecked( false );

    m_fastCheckCheckBox = new QCheckBox( tr( "Fast check" ), this );
    m_layout->addWidget( m_fastCheckCheckBox );
    m_fastCheckCheckBox->setChecked( true );

    m_layout->addStretch();

    m_rescaleCheckBox = new QCheckBox( tr( "Rescale preview" ), this );
    m_layout->addWidget( m_rescaleCheckBox );
    m_rescaleCheckBox->setChecked( true );

    QHBoxLayout *rescaleLayout = new QHBoxLayout();

    m_rescaleSizeSpinBox = new RescaleSpinBox( this );

    rescaleLayout->addWidget( new QLabel( tr( "Maximum size:" ) ) );
    rescaleLayout->addWidget( m_rescaleSizeSpinBox );

    m_layout->addLayout( rescaleLayout );

    connect( m_typeCombo, static_cast< void ( TypeComboBox::* )( int ) >( &TypeComboBox::currentIndexChanged ), this, &CameraParametersWidget::parametersChanges );
    connect( m_xCountSpinBox, static_cast< void ( CountSpinBox::* )( int ) >( &CountSpinBox::valueChanged ), this, &CameraParametersWidget::parametersChanges );
    connect( m_yCountSpinBox, static_cast< void ( CountSpinBox::* )( int ) >( &CountSpinBox::valueChanged ), this, &CameraParametersWidget::parametersChanges );
    connect( m_sizeSpinBox, static_cast< void ( SizeSpinBox::* )( double ) >( &SizeSpinBox::valueChanged ), this, &CameraParametersWidget::parametersChanges );

    connect( m_adaptiveThresholdCheckBox, &QCheckBox::stateChanged , this, &CameraParametersWidget::parametersChanges );
    connect( m_normalizeImageCheckBox, &QCheckBox::stateChanged , this, &CameraParametersWidget::parametersChanges );
    connect( m_filterQuadsCheckBox, &QCheckBox::stateChanged , this, &CameraParametersWidget::parametersChanges );
    connect( m_fastCheckCheckBox, &QCheckBox::stateChanged , this, &CameraParametersWidget::parametersChanges );

    connect( m_rescaleCheckBox, &QCheckBox::stateChanged , this, &CameraParametersWidget::parametersChanges );
    connect( m_rescaleSizeSpinBox, static_cast< void ( RescaleSpinBox::* )( int ) >( &RescaleSpinBox::valueChanged ), this, &CameraParametersWidget::parametersChanges );

}

bool CameraParametersWidget::adaptiveThreshold() const
{
    return m_adaptiveThresholdCheckBox->isChecked();
}

bool CameraParametersWidget::normalizeImage() const
{
    return m_normalizeImageCheckBox->isChecked();
}

bool CameraParametersWidget::filterQuads() const
{
    return m_filterQuadsCheckBox->isChecked();
}

bool CameraParametersWidget::fastCheck() const
{
    return m_fastCheckCheckBox->isChecked();
}

bool CameraParametersWidget::rescaleFlag() const
{
    return m_rescaleCheckBox->isChecked();
}

unsigned int CameraParametersWidget::rescaleSize() const
{
    return static_cast< unsigned int >( m_rescaleSizeSpinBox->value() );
}
