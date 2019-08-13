#include "src/common/precompiled.h"

#include "fileslistwidget.h"

// MonocularFilesListWidget
MonocularFilesListWidget::MonocularFilesListWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void MonocularFilesListWidget::initialize()
{
    m_layout = new QVBoxLayout( this );

    m_toolBar = new QToolBar( tr("File toolbar") );

    m_addAction = m_toolBar->addAction( QIcon( ":/resources/images/plus.ico" ), tr( "Add images" ) );
    m_removeAction = m_toolBar->addAction( QIcon( ":/resources/images/remove.ico" ), tr( "Remove images" ) );

    connect( m_addAction, &QAction::triggered, this, &MonocularFilesListWidget::addDialog );
    connect( m_removeAction, &QAction::triggered, this, &MonocularFilesListWidget::removeDialog );

    m_layout->addWidget( m_toolBar );

    m_list = new FilesListWidget( this );

    m_layout->addWidget( m_list );

}

int MonocularFilesListWidget::count() const
{
    return m_list->count();
}

QStringList MonocularFilesListWidget::fileNames() const
{
    QStringList ret;

    for ( auto i = 0; i < count(); ++i )
        ret.push_back( m_list->item( i )->text() );

    return ret;
}

void MonocularFilesListWidget::addDialog()
{
    auto files = QFileDialog::getOpenFileNames(
                            this,
                            tr( "Select images for calibration" ),
                            QString(),
                            "Image files (*.png *.xpm *.jpg)" );

    m_list->addItems( files );

}

void MonocularFilesListWidget::removeDialog()
{
    if ( QMessageBox::question( this, tr( "Remove items" ), tr( "Do you want to remove selected items?" ) ) == QMessageBox::Yes )
        m_list->removeSelected();

}

// StereoFilesListWidget
StereoFilesListWidget::StereoFilesListWidget( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void StereoFilesListWidget::initialize()
{
    m_leftList = new MonocularFilesListWidget( this );
    m_rightList = new MonocularFilesListWidget( this );

    addWidget( m_leftList );
    addWidget( m_rightList );
}

int StereoFilesListWidget::leftCount() const
{
    return m_leftList->count();
}

int StereoFilesListWidget::rightCount() const
{
    return m_rightList->count();
}

QStringList StereoFilesListWidget::leftFileNames() const
{
    return m_leftList->fileNames();
}

QStringList StereoFilesListWidget::rightFileNames() const
{
    return m_rightList->fileNames();
}

// StereoFilesListDialog
StereoFilesListDialog::StereoFilesListDialog( QWidget *parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void StereoFilesListDialog::initialize()
{
    setWidget( new StereoFilesListWidget( this) );

    resize( 600, 700 );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &StereoFilesListDialog::onAccept );
}

StereoFilesListWidget *StereoFilesListDialog::widget() const
{
    return dynamic_cast< StereoFilesListWidget * >( m_widget.data() );
}

int StereoFilesListDialog::leftCount() const
{
    return widget()->leftCount();
}

int StereoFilesListDialog::rightCount() const
{
    return widget()->rightCount();
}

QStringList StereoFilesListDialog::leftFileNames() const
{
    return widget()->leftFileNames();
}

QStringList StereoFilesListDialog::rightFileNames() const
{
    return widget()->rightFileNames();
}

void StereoFilesListDialog::onAccept()
{
    auto widget = this->widget();

    if ( widget->leftCount() != widget->rightCount() ) {
        QMessageBox::information( this, tr( "Error" ), tr( "Left image count must be equal to right image count!" ) );
    }
    else
        accept();

}

// StereoDirsWidget
StereoDirWidget::StereoDirWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void StereoDirWidget::initialize()
{
    auto layout = new QVBoxLayout( this );

    m_leftFolderWidget = new FolderLine( tr( "Left folder:" ), this );
    layout->addWidget( m_leftFolderWidget );

    m_rightFolderWidget = new FolderLine( tr( "Right folder:" ), this );
    layout->addWidget( m_rightFolderWidget );

}

QString StereoDirWidget::leftDir() const
{
    return m_leftFolderWidget->path();
}

QString StereoDirWidget::rightDir() const
{
    return m_rightFolderWidget->path();
}

// StereoDirDialog
StereoDirDialog::StereoDirDialog( QWidget *parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void StereoDirDialog::initialize()
{
    setWidget( new StereoDirWidget( this ) );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &StereoDirDialog::accept );
}

StereoDirWidget *StereoDirDialog::widget() const
{
    return dynamic_cast< StereoDirWidget * >( m_widget.data() );
}

QString StereoDirDialog::leftDir() const
{
    return widget()->leftDir();
}

QString StereoDirDialog::rightDir() const
{
    return widget()->rightDir();
}


