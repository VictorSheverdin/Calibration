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
    setWidget( new StereoDirWidget( tr("Left images directory"), tr("Right images directory"), this ) );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &StereoDirDialog::accept );
}

StereoDirWidget *ImagesDialog::widget() const
{
    return dynamic_cast< StereoDirWidget * >( m_widget.data() );
}

QString ImagesDialog::leftDir() const
{
    return widget()->leftDir();
}

QString ImagesDialog::rightDir() const
{
    return widget()->rightDir();
}

// StereoIPDialog
CamerasDialog::CamerasDialog( QWidget* parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void CamerasDialog::initialize()
{
    setWidget( new StereoIPWidget( this ) );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &DialogBase::accept );
}

StereoIPWidget *CamerasDialog::widget() const
{
    return dynamic_cast< StereoIPWidget * >( m_widget.data() );
}

QString CamerasDialog::leftIp() const
{
    return widget()->leftIp();
}

QString CamerasDialog::rightIp() const
{
    return widget()->rightIp();
}
