#include "src/common/precompiled.h"

#include "disparitychoicedialog.h"

// DisparityChoiceDialog
DisparityChoiceDialog::DisparityChoiceDialog( QWidget* parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void DisparityChoiceDialog::initialize()
{
    setWindowTitle( tr("Choice document type") );

    m_listWidget = new QListWidget( this );

    auto imagesDocument = new QListWidgetItem( tr("Disparity from images") );
    imagesDocument->setData( Qt::UserRole, IMAGES );

    auto cameraDocument = new QListWidgetItem( tr("Disparity from camera") );
    cameraDocument->setData( Qt::UserRole, CAMERA );

    m_listWidget->addItem( imagesDocument );
    m_listWidget->addItem( cameraDocument );

    m_listWidget->setSelectionMode( QListWidget::SingleSelection );

    imagesDocument->setSelected( true );

    setWidget( m_listWidget );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &DialogBase::accept );

}

DisparityChoiceDialog::DocumentType DisparityChoiceDialog::selectedType() const
{
    auto selectedList = m_listWidget->selectedItems();

    if ( selectedList.empty() )
        return NONE;
    else
        return static_cast< DisparityChoiceDialog::DocumentType >( selectedList.front()->data( Qt::UserRole ).toInt() );
}
