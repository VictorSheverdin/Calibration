#include "src/common/precompiled.h"

#include "choicedialog.h"

// ChoiceDialog
ChoiceDialog::ChoiceDialog( QWidget* parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void ChoiceDialog::initialize()
{
    setWindowTitle( tr("Choice document type") );

    m_listWidget = new QListWidget( this );

    auto imagesDocument = new QListWidgetItem( tr("Image sequence") );
    imagesDocument->setData( Qt::UserRole, IMAGES );

    auto cameraDocument = new QListWidgetItem( tr("Camera capture") );
    cameraDocument->setData( Qt::UserRole, CAMERA );

    m_listWidget->addItem( imagesDocument );
    m_listWidget->addItem( cameraDocument );

    m_listWidget->setSelectionMode( QListWidget::SingleSelection );

    imagesDocument->setSelected( true );

    setWidget( m_listWidget );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &DialogBase::accept );

}

ChoiceDialog::DocumentType ChoiceDialog::selectedType() const
{
    auto selectedList = m_listWidget->selectedItems();

    if ( selectedList.empty() )
        return NONE;
    else
        return static_cast< ChoiceDialog::DocumentType >( selectedList.front()->data( Qt::UserRole ).toInt() );
}
