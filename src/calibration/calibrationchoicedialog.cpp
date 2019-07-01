#include "src/common/precompiled.h"

#include "calibrationchoicedialog.h"

// CalibrationChoiceDialog
CalibrationChoiceDialog::CalibrationChoiceDialog( QWidget* parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void CalibrationChoiceDialog::initialize()
{
    setWindowTitle( tr("Choice document type") );

    m_listWidget = new QListWidget( this );

    auto monocularImageDocument = new QListWidgetItem( tr("Monocular calibration from images") );
    monocularImageDocument->setData( Qt::UserRole, MONOCULAR_IMAGE );

    auto stereoImageDocument = new QListWidgetItem( tr("Stereo calibration from images") );
    stereoImageDocument->setData( Qt::UserRole, STEREO_IMAGE );

    auto monocularCameraDocument = new QListWidgetItem( tr("Monocular calibration from camera") );
    monocularCameraDocument->setData( Qt::UserRole, MONOCULAR_CAMERA );

    auto stereoCameraDocument = new QListWidgetItem( tr("Stereo calibration from camera") );
    stereoCameraDocument->setData( Qt::UserRole, STEREO_CAMERA );

    m_listWidget->addItem( monocularImageDocument );
    m_listWidget->addItem( stereoImageDocument );
    m_listWidget->addItem( monocularCameraDocument );
    m_listWidget->addItem( stereoCameraDocument );

    m_listWidget->setSelectionMode( QListWidget::SingleSelection );

    monocularImageDocument->setSelected( true );

    setWidget( m_listWidget );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &DialogBase::accept );

}

CalibrationChoiceDialog::DocumentType CalibrationChoiceDialog::selectedType() const
{
    auto selectedList = m_listWidget->selectedItems();

    if ( selectedList.empty() )
        return NONE;
    else
        return static_cast< CalibrationChoiceDialog::DocumentType >( selectedList.front()->data( Qt::UserRole ).toInt() );
}
