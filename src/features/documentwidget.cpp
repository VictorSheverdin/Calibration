#include "src/common/precompiled.h"

#include "documentwidget.h"

#include "viewwidget.h"

// CalibrationDocumentBase
FeaturesDocument::FeaturesDocument( QWidget* parent )
    : DocumentBase( parent )
{
    initialize();
}

void FeaturesDocument::initialize()
{
    setWidget( new FeaturesWidget( this ) );
}

FeaturesWidget *FeaturesDocument::widget() const
{
    return dynamic_cast< FeaturesWidget * >( m_widget );
}

void FeaturesDocument::importDialog()
{
    widget()->importDialog();
}
