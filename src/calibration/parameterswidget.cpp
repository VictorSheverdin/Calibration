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
    addItem( tr( "Checkerboard" ), CHECKERBOARD );
    addItem( tr( "Circles" ), CIRCLES );
    addItem( tr( "Asymetric circles" ), ASYM_CIRCLES );
    addItem( tr( "Aruco markers" ), ARUCO_MARKERS );
}

TypeComboBox::Type TypeComboBox::currentType() const
{
    return static_cast< Type >( currentData().toInt() );
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

    m_countLabel = new QLabel( tr( "Count:" ) );
    countLayout->addWidget( m_countLabel );

    m_xCountSpinBox = new CountSpinBox( this );
    m_xCountSpinBox->setValue( 9 );
    m_yCountSpinBox = new CountSpinBox( this );
    m_yCountSpinBox->setValue( 6 );

    countLayout->addWidget( m_xCountSpinBox );
    countLayout->addWidget( m_yCountSpinBox );

    m_layout->addLayout( countLayout );

    QHBoxLayout *sizeLayout = new QHBoxLayout();

    m_sizeLabel = new QLabel( tr( "Size:" ) );
    sizeLayout->addWidget( m_sizeLabel );

    m_sizeSpinBox = new SizeSpinBox( this );
    m_sizeSpinBox->setValue( 0.05 );
    m_sizeSpinBox->setDecimals( 3 );
    sizeLayout->addWidget( m_sizeSpinBox );

    m_sizeMeasLabel = new QLabel( tr( "m. " ) );
    sizeLayout->addWidget( m_sizeMeasLabel );

    m_intervalLabel = new QLabel( tr( "Interval:" ) );
    sizeLayout->addWidget( m_intervalLabel );

    m_intervalSpinBox = new SizeSpinBox( this );
    m_intervalSpinBox->setValue( 0.05 );
    m_intervalSpinBox->setDecimals( 3 );
    sizeLayout->addWidget( m_intervalSpinBox );

    m_intervalMeasLabel = new QLabel( tr( "m. " ) );
    sizeLayout->addWidget( m_intervalMeasLabel );

    m_layout->addLayout( sizeLayout );

    m_layout->addStretch();

    connect( m_typeCombo, static_cast< void ( TypeComboBox::* )( int ) >( &TypeComboBox::currentIndexChanged ), this, &ParametersWidget::updateVisibility );

    updateVisibility();

}

TypeComboBox::Type ParametersWidget::templateType() const
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

double ParametersWidget::intervalSize() const
{
    return m_intervalSpinBox->value();
}

void ParametersWidget::updateVisibility()
{
    auto templateType = this->templateType();

    if ( templateType == TypeComboBox::CHECKERBOARD || templateType == TypeComboBox::CIRCLES || templateType == TypeComboBox::ASYM_CIRCLES ) {
        m_countLabel->setVisible( true );
        m_xCountSpinBox->setVisible( true );
        m_yCountSpinBox->setVisible( true );
        m_intervalLabel->setVisible( false );
        m_intervalSpinBox->setVisible( false );
        m_intervalMeasLabel->setVisible( false );
    }
    else if ( templateType == TypeComboBox::ARUCO_MARKERS ) {
        m_countLabel->setVisible( false );
        m_xCountSpinBox->setVisible( false );
        m_yCountSpinBox->setVisible( false );
        m_intervalLabel->setVisible( true );
        m_intervalSpinBox->setVisible( true );
        m_intervalMeasLabel->setVisible( true );
    }

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
    rescaleLayout->addWidget( new QLabel( tr( "pix. " ) ) );

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
