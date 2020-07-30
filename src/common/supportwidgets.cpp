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
DocumentBase::DocumentBase( QWidget *parent )
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

    setupStatusBar();
}

void MainWindowBase::setupStatusBar()
{
    m_statusBar = new QStatusBar( this );
    setStatusBar( m_statusBar );

}

// DocumentMainWindow
DocumentMainWindow::DocumentMainWindow( QWidget *parent )
    : MainWindowBase( parent )
{
    initialize();
}

void DocumentMainWindow::initialize()
{
    setupDocuments();

}

void DocumentMainWindow::addDocument( DocumentBase *document )
{
    m_documentArea->addDocument( document );
}

DocumentBase *DocumentMainWindow::currentDocument() const
{
    return m_documentArea->currentDocument();
}

void DocumentMainWindow::setupDocuments()
{
    m_documentArea = new DocumentArea( this );

    setCentralWidget( m_documentArea );    
}

// FileLineCollection
FileLineCollection::FileLineCollection()
{
}

void FileLineCollection::initialize( QWidget *parent )
{
    m_label = new QLabel( parent );
    m_path = new QLineEdit( parent );
    m_button = new QPushButton( QIcon( ":/resources/images/folder.ico" ), QString(), parent );
}

void FileLineCollection::setLabel( const QString &label )
{
    m_label->setText( label );
}

QString FileLineCollection::label() const
{
    return m_label->text();
}

void FileLineCollection::setPath( const QString &value )
{
    m_path->setText( value );
}

QString FileLineCollection::path() const
{
    return m_path->text();
}

// FileLineBase
FileLineBase::FileLineBase( QWidget *parent )
    : QWidget( parent )
{
    initialize();
}

FileLineBase::FileLineBase( const QString &label, QWidget *parent )
    : QWidget( parent )
{
    initialize();

    setLabel( label );
}

void FileLineBase::initialize()
{
    setContentsMargins( QMargins() );

    auto layout = new QHBoxLayout( this );

    layout->setContentsMargins( QMargins() );

    FileLineCollection::initialize( this );

    layout->addWidget( m_label );
    layout->addWidget( m_path );
    layout->addWidget( m_button );
}

// FileLine
FileLine::FileLine( const QString &filter, QWidget *parent )
    : FileLineBase( parent ), m_filter( filter )
{
    initialize();
}

FileLine::FileLine( const QString &label, const QString &filter, QWidget *parent )
    : FileLineBase( label, parent ), m_filter( filter )
{
    initialize();
}

void FileLine::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Minimum );

    connect( m_button, &QPushButton::clicked, this, &FileLine::choiceFileDialog );
}

void FileLine::choiceFileDialog()
{
    auto dir = QFileDialog::getOpenFileName( this, tr( "Open File" ), QString(), m_filter );

    if ( !dir.isEmpty() )
        setPath( dir );

}

// FolderLine
FolderLine::FolderLine( QWidget *parent )
    : FileLineBase( parent )
{
    initialize();
}

FolderLine::FolderLine( const QString &label, QWidget *parent )
    : FileLineBase( label, parent )
{
    initialize();
}

void FolderLine::initialize()
{
    connect( m_button, &QPushButton::clicked, this, &FolderLine::choiceDirectoryDialog );
}

void FolderLine::choiceDirectoryDialog()
{
    auto dir = QFileDialog::getExistingDirectory( this, tr( "Open Directory" ), QString(), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks );

    if ( !dir.isEmpty() )
        setPath( dir );

}


