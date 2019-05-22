#include "src/common/precompiled.h"

#include "supportwidgets.h"
#include "src/common/defs.h"

#include "documentarea.h"

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

// FilesListWidget
FilesListWidget::FilesListWidget( QWidget *parent )
    : QListWidget( parent )
{
    initialize();
}

void FilesListWidget::initialize()
{
    setSelectionMode( ExtendedSelection );
}

void FilesListWidget::removeSelected()
{
    auto selected = selectedIndexes();

    std::set< int > sorted;

    for ( auto &i : selected )
        sorted.insert( i.row() );

    for ( auto i = sorted.rbegin(); i != sorted.rend(); ++i )
        takeItem( *i );
}

// DocumentBase
DocumentBase::DocumentBase( QWidget *widget, QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void DocumentBase::initialize()
{
    m_layout = new QVBoxLayout( this );
}

void DocumentBase::setWidget( QWidget *widget )
{
    m_widget = widget;

    m_layout->addWidget( widget );
}

QWidget *DocumentBase::widget() const
{
    return m_widget;
}

// DialogBase
DialogBase::DialogBase( QWidget *parent )
    : QDialog( parent )
{
    initialize();
}

DialogBase::DialogBase( const QDialogButtonBox::StandardButtons buttons, QWidget *parent )
    : QDialog( parent )
{
    initialize( buttons );
}

void DialogBase::initialize()
{
    m_layout = new QVBoxLayout( this );
}

void DialogBase::initialize( const QDialogButtonBox::StandardButtons buttons )
{
    initialize();

    m_buttons = new QDialogButtonBox( buttons, Qt::Horizontal, this );
    m_layout->addWidget( m_buttons );

    connect( m_buttons, &QDialogButtonBox::rejected, this, &DialogBase::reject );

}

void DialogBase::setWidget( QWidget *widget )
{
    if ( !m_widget && widget ) {
        m_widget = widget;
        m_layout->insertWidget( 0, widget );
    }
}

// MainWindowBase
MainWindowBase::MainWindowBase( QWidget *parent )
    : QMainWindow( parent )
{
    initialize();
}

void MainWindowBase::initialize()
{
    setAttribute( Qt::WA_DeleteOnClose );

    setupDocuments();
    setupStatusBar();

}

void MainWindowBase::addDocument( DocumentBase *document )
{
    m_documentArea->addDocument( document );
}

DocumentBase *MainWindowBase::currentDocument() const
{
    m_documentArea->currentDocument();
}

void MainWindowBase::setupDocuments()
{
    m_documentArea = new DocumentArea( this );

    setCentralWidget( m_documentArea );
}

void MainWindowBase::setupStatusBar()
{
    m_statusBar = new QStatusBar( this );
    setStatusBar( m_statusBar );

}
