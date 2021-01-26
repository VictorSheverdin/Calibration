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

    _calibrationFileLine = new FileLine( tr( "Calibration file:" ), tr( "Calibration files (*.yaml)" ), this );
    layout->addWidget( _calibrationFileLine );

    _filesListWidget = new StereoFilesListWidget( this );
    layout->addWidget( _filesListWidget );

}

int ImagesChoiceWidget::leftCount() const
{
    return _filesListWidget->leftCount();
}

int ImagesChoiceWidget::rightCount() const
{
    return _filesListWidget->rightCount();
}

QStringList ImagesChoiceWidget::leftFileNames() const
{
    return _filesListWidget->leftFileNames();
}

QStringList ImagesChoiceWidget::rightFileNames() const
{
    return _filesListWidget->rightFileNames();
}

QString ImagesChoiceWidget::calibrationFile() const
{
    return _calibrationFileLine->path();
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

    connect( m_buttons, &QDialogButtonBox::accepted, this, &StereoDirDialog::accept );
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

QString ImagesDialog::calibrationFile() const
{
    return widget()->calibrationFile();
}
