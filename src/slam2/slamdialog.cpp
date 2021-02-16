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

    _leftMaskLine = new FileLine( QString(), tr( "Image files (*.png *.xpm *.jpg *.tiff *.exr)" ), this );
    _rightMaskLine = new FileLine( QString(), tr( "Image files (*.png *.xpm *.jpg *.tiff *.exr)" ), this );

    QFormLayout *maskLayout = new QFormLayout();

    maskLayout->addRow( new QLabel( tr( "left mask file:" ) ), _leftMaskLine );
    maskLayout->addRow( new QLabel( tr( "Right mask file:" ) ), _rightMaskLine );

    layout->addLayout( maskLayout );

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

QString ImagesChoiceWidget::leftMaskFile() const
{
    return _leftMaskLine->path();
}

QString ImagesChoiceWidget::rightMaskFile() const
{
    return _rightMaskLine->path();
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

QString ImagesDialog::leftMaskFile() const
{
    return widget()->leftMaskFile();
}

QString ImagesDialog::rightMaskFile() const
{
    return widget()->rightMaskFile();
}
