#include "src/common/precompiled.h"

#include "controlwidget.h"

// DetectorComboBox
DetectorComboBox::DetectorComboBox( QWidget *parent )
    : QComboBox( parent )
{
    initialize();
}

void DetectorComboBox::initialize()
{
    addItem( tr( "Shi-Tomasi" ), Type::ShiTomasi );
    addItem( tr( "FAST" ), Type::FAST );
    addItem( tr( "SIFT" ), Type::SIFT );
    addItem( tr( "SURF" ), Type::SURF );
    addItem( tr( "ORB" ), Type::ORB );
    addItem( tr( "KAZE" ), Type::KAZE );
    addItem( tr( "AKAZE" ), Type::AKAZE );
    addItem( tr( "Super Glue" ), Type::SUPERGLUE );
}

DetectorComboBox::Type DetectorComboBox::currentType() const
{
    return static_cast<Type>( currentData().toInt() );
}

void DetectorComboBox::setCurrentType( const Type value )
{
    for ( auto i = 0; i < count(); ++i )
        if ( static_cast< Type >( itemData( i ).toInt() ) == value ) {
            setCurrentIndex( i );
            return;
        }
}

// DescriptorComboBox
DescriptorComboBox::DescriptorComboBox( QWidget *parent )
    : QComboBox( parent )
{
    initialize();
}

void DescriptorComboBox::initialize()
{
    addItem( tr( "LK Flow" ), Type::LK );
    addItem( tr( "DAISY" ), Type::DAISY );
    addItem( tr( "FREAK" ), Type::FREAK );
    addItem( tr( "SIFT" ), Type::SIFT );
    addItem( tr( "SURF" ), Type::SURF );
    addItem( tr( "ORB" ), Type::ORB );
    addItem( tr( "KAZE" ), Type::KAZE );
    addItem( tr( "AKAZE" ), Type::AKAZE );
    addItem( tr( "Super Glue" ), Type::SUPERGLUE );
}

DescriptorComboBox::Type DescriptorComboBox::currentType() const
{
    return static_cast< Type >( currentData().toInt() );
}

void DescriptorComboBox::setCurrentType( const Type value )
{
    for ( auto i = 0; i < count(); ++i )
        if ( static_cast< Type >( itemData( i ).toInt() ) == value ) {
            setCurrentIndex( i );
            return;
        }
}

// DetectorLayout
DetectorLayout::DetectorLayout( QWidget* parent )
    : QHBoxLayout( parent )
{
    initialize();
}

void DetectorLayout::initialize()
{
    m_label = new QLabel( tr("Detector type") );
    addWidget( m_label );

    m_typeComboBox = new DetectorComboBox();
    addWidget( m_typeComboBox );

    connect( m_typeComboBox, static_cast< void ( DetectorComboBox::* )( int ) >( &DetectorComboBox::currentIndexChanged ), this, &DetectorLayout::currentIndexChanged );
}

DetectorComboBox::Type DetectorLayout::value() const
{
    return m_typeComboBox->currentType();
}

void DetectorLayout::setValue( const DetectorComboBox::Type value )
{
    m_typeComboBox->setCurrentType( value );
}

// DescriptorLayout
DescriptorLayout::DescriptorLayout( QWidget* parent )
    : QHBoxLayout( parent )
{
    initialize();
}

void DescriptorLayout::initialize()
{
    m_label = new QLabel( tr("Descriptor type") );
    addWidget( m_label );

    m_typeComboBox = new DescriptorComboBox();
    addWidget( m_typeComboBox );

    connect( m_typeComboBox, static_cast< void ( DescriptorComboBox::* )( int ) >( &DescriptorComboBox::currentIndexChanged ), this, &DescriptorLayout::currentIndexChanged );
}

DescriptorComboBox::Type DescriptorLayout::value() const
{
    return m_typeComboBox->currentType();
}

void DescriptorLayout::setValue( const DescriptorComboBox::Type value )
{
    m_typeComboBox->setCurrentType( value );
}

// FeaturesControlWidget
FeaturesControlWidget::FeaturesControlWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void FeaturesControlWidget::initialize()
{
    setSizePolicy( QSizePolicy::Minimum, QSizePolicy::Minimum );

    auto layout = new QVBoxLayout( this );

    auto scaleFactorLayout = new QHBoxLayout();

    auto scaleFactorText = new QLabel(tr( "Scale factor:" ), this );

    m_scaleFactorSpinBox = new QDoubleSpinBox( this );
    m_scaleFactorSpinBox->setMinimum( .1 );
    m_scaleFactorSpinBox->setMaximum( 3 );
    m_scaleFactorSpinBox->setSingleStep( .1 );
    m_scaleFactorSpinBox->setValue( 1. );

    scaleFactorLayout->addWidget( scaleFactorText );
    scaleFactorLayout->addWidget( m_scaleFactorSpinBox );

    layout->addLayout( scaleFactorLayout );

    m_detectorLayout = new DetectorLayout();
    layout->addLayout( m_detectorLayout );

    m_detectorStack = new QStackedWidget( this );
    layout->addWidget( m_detectorStack );

    m_descriptorLayout = new DescriptorLayout();
    layout->addLayout( m_descriptorLayout );

    m_descriptorStack = new QStackedWidget( this );
    layout->addWidget( m_descriptorStack );

    connect( m_scaleFactorSpinBox, static_cast< void ( QDoubleSpinBox::* )( double ) >( &QDoubleSpinBox::valueChanged ), this, &FeaturesControlWidget::valueChanged );

    connect( m_detectorLayout, &DetectorLayout::currentIndexChanged, this, &FeaturesControlWidget::updateStackedWidget );
    connect( m_descriptorLayout, &DescriptorLayout::currentIndexChanged, this, &FeaturesControlWidget::updateStackedWidget );

    connect( m_detectorLayout, &DetectorLayout::currentIndexChanged, this, &FeaturesControlWidget::checkValues );
    connect( m_descriptorLayout, &DescriptorLayout::currentIndexChanged, this, &FeaturesControlWidget::checkValues );

    updateStackedWidget();
}

double FeaturesControlWidget::scaleFactor() const
{
    return m_scaleFactorSpinBox->value();
}

bool FeaturesControlWidget::isShiTomasiDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::ShiTomasi;
}

bool FeaturesControlWidget::isFastDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::FAST;
}

bool FeaturesControlWidget::isSiftDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::SIFT;
}

bool FeaturesControlWidget::isSurfDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::SURF;
}

bool FeaturesControlWidget::isOrbDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::ORB;
}

bool FeaturesControlWidget::isKazeDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::KAZE;
}

bool FeaturesControlWidget::isAkazeDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::AKAZE;
}

bool FeaturesControlWidget::isSuperGlueDetector() const
{
    return m_detectorLayout->value() == DetectorComboBox::SUPERGLUE;
}

bool FeaturesControlWidget::isLKFlow() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::LK;
}

bool FeaturesControlWidget::isDaisyDescriptor() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::DAISY;
}

bool FeaturesControlWidget::isFreakDescriptor() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::FREAK;
}

bool FeaturesControlWidget::isSiftDescriptor() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::SIFT;
}

bool FeaturesControlWidget::isSurfDescriptor() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::SURF;
}

bool FeaturesControlWidget::isKazeDescriptor() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::KAZE;
}

bool FeaturesControlWidget::isAkazeDescriptor() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::AKAZE;
}

bool FeaturesControlWidget::isSuperGlueDescriptor() const
{
    return m_descriptorLayout->value() == DescriptorComboBox::SUPERGLUE;
}

void FeaturesControlWidget::checkValues()
{
    if ( m_descriptorLayout->value() == DescriptorComboBox::KAZE )
        m_detectorLayout->setValue( DetectorComboBox::KAZE );
    else if ( m_descriptorLayout->value() == DescriptorComboBox::AKAZE )
        m_detectorLayout->setValue( DetectorComboBox::AKAZE );

    emit valueChanged();
}

void FeaturesControlWidget::updateStackedWidget()
{
}
