#include "precompiled.h"

#include "supportwidgets.h"

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
}

TemplateProcessor::Type TypeComboBox::currentType() const
{
    return static_cast<TemplateProcessor::Type>( currentData().toInt() );
}

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

    setValue( 800 );

    setAlignment( Qt::AlignRight );

}
