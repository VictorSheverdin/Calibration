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

    m_calibrationFileLine = new FileLine( tr( "Calibration file:" ), tr( "Calibration files (*.yaml)" ), this );
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

QString ImagesChoiceWidget::calibrationFile() const
{
    return m_calibrationFileLine->path();
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

// CamerasChoiceWidget
CamerasChoiceWidget::CamerasChoiceWidget( QWidget *parent )
    : QWidget( parent )
{
    initialize();
}

void CamerasChoiceWidget::initialize()
{
    QVBoxLayout *layout = new QVBoxLayout( this );

    m_calibrationFileLine = new FileLine( tr( "Calibration file:" ), tr( "Calibration files (*.yaml)" ), this );
    layout->addWidget( m_calibrationFileLine );

    m_camerasIpWidget = new StereoIPWidget( this );
    layout->addWidget( m_camerasIpWidget );
}

QString CamerasChoiceWidget::leftIp() const
{
    return m_camerasIpWidget->leftIp();
}

QString CamerasChoiceWidget::rightIp() const
{
    return m_camerasIpWidget->rightIp();
}

QString CamerasChoiceWidget::calibrationFile() const
{
    return m_calibrationFileLine->path();
}

// StereoIPDialog
CamerasDialog::CamerasDialog( QWidget* parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void CamerasDialog::initialize()
{
    setWidget( new CamerasChoiceWidget( this ) );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &DialogBase::accept );
}

CamerasChoiceWidget *CamerasDialog::widget() const
{
    return dynamic_cast< CamerasChoiceWidget * >( m_widget.data() );
}

QString CamerasDialog::leftIp() const
{
    return widget()->leftIp();
}

QString CamerasDialog::rightIp() const
{
    return widget()->rightIp();
}

QString CamerasDialog::calibrationFile() const
{
    return widget()->calibrationFile();
}
