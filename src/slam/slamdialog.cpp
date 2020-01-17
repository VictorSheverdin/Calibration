#include "src/common/precompiled.h"

#include "slamdialog.h"

// ImagesChoiceWidget
ImagesChoiceWidget::ImagesChoiceWidget( QWidget *parent )
    : QWidget( parent )
{
    initialize();
}

void ImagesChoiceWidget::initialize()
{
    QVBoxLayout *layout = new QVBoxLayout( this );

    m_calibrationFileLine = new FileLine( this );
    layout->addWidget( m_calibrationFileLine );

    m_filesListWidget = new StereoFilesListWidget( this );
    layout->addWidget( m_filesListWidget );

}

int ImagesChoiceWidget::leftCount() const
{
    return m_filesListWidget->leftCount();
}

int ImagesChoiceWidget::rightCount() const
{
    return m_filesListWidget->rightCount();
}

QStringList ImagesChoiceWidget::leftFileNames() const
{
    return m_filesListWidget->leftFileNames();
}

QStringList ImagesChoiceWidget::rightFileNames() const
{
    return m_filesListWidget->rightFileNames();
}

// ImagesDialog
ImagesDialog::ImagesDialog( QWidget *parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void ImagesDialog::initialize()
{
    setWidget( new ImagesChoiceWidget( this ) );

    resize( 600, 700 );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &ImagesDialog::onAccept );
}

ImagesChoiceWidget *ImagesDialog::widget() const
{
    return dynamic_cast< ImagesChoiceWidget * >( m_widget.data() );
}

int ImagesDialog::leftCount() const
{
    return widget()->leftCount();
}

int ImagesDialog::rightCount() const
{
    return widget()->rightCount();
}

QStringList ImagesDialog::leftFileNames() const
{
    return widget()->leftFileNames();
}

QStringList ImagesDialog::rightFileNames() const
{
    return widget()->rightFileNames();
}

void ImagesDialog::onAccept()
{
    auto widget = this->widget();

    if ( widget->leftCount() != widget->rightCount() ) {
        QMessageBox::information( this, tr( "Error" ), tr( "Left image count must be equal to right image count!" ) );
    }
    else
        accept();

}
