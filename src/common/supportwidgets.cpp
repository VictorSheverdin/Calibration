#include "src/common/precompiled.h"

#include "supportwidgets.h"
#include "src/common/defs.h"

// CountSpinBox
CountSpinBox::CountSpinBox( QWidget *parent )
    : QSpinBox( parent )
{
    initialize();
}

void CountSpinBox::initialize()
{
    setMinimum( 3 );
    setMaximum( 100 );

    setValue( 5 );

    setAlignment( Qt::AlignRight );

}

// SizeSpinBox
SizeSpinBox::SizeSpinBox( QWidget *parent )
    : QDoubleSpinBox( parent )
{
    initialize();
}

void SizeSpinBox::initialize()
{
    setMinimum( 0 );
    setMaximum( 1e6 );

    setValue( 1.0 );

    setAlignment( Qt::AlignRight );

}

// RescaleSpinBox
RescaleSpinBox::RescaleSpinBox( QWidget *parent )
    : QSpinBox( parent )
{
    initialize();
}

void RescaleSpinBox::initialize()
{
    setMinimum( 300 );
    setMaximum( 4000 );

    setValue( 600 );

    setAlignment( Qt::AlignRight );

}

// IntSliderBox
IntSliderBox::IntSliderBox( const QString label, QWidget* parent )
    : QWidget( parent )
{
    initialize( label );
}

void IntSliderBox::initialize( const QString label )
{
    m_stepSize = 1;

    m_layout = new QHBoxLayout( this );

    m_label = new QLabel( label, this );
    m_slider = new QSlider( Qt::Horizontal, this );
    m_displayWidget = new QLineEdit( this );
    m_displayWidget->setReadOnly( true );
    m_displayWidget->setAlignment( Qt::AlignVCenter | Qt::AlignRight );
    m_displayWidget->setFixedWidth( 50 );

    m_layout->addWidget( m_label );
    m_layout->addWidget( m_slider );

    m_layout->addWidget( m_displayWidget );

    connect( m_slider, &QSlider::valueChanged, this, &IntSliderBox::updateDisplayedValue );

    updateDisplayedValue();

}


int IntSliderBox::value()
{
    return m_slider->minimum() + ( m_slider->value() - m_slider->minimum() ) * m_stepSize;
}

int IntSliderBox::stepSize() const
{
    return m_stepSize;
}

int IntSliderBox::minimum() const
{
    return m_slider->minimum();
}

int IntSliderBox::maximum() const
{
    return m_slider->minimum() + ( m_slider->maximum() - m_slider->maximum() ) * m_stepSize;
}

void IntSliderBox::setRange( const int minValue, const int maxValue, const int step )
{
    m_stepSize = std::max( 1, step );

    m_slider->setMinimum( minValue );
    m_slider->setMaximum( minValue + ( maxValue - minValue ) / m_stepSize );

}

void IntSliderBox::setValue( const int value )
{
    m_slider->setValue( m_slider->minimum() + ( value - m_slider->minimum() ) / m_stepSize );
}

void IntSliderBox::updateDisplayedValue()
{
    m_displayWidget->setText( QString::number( value() ) );
}
