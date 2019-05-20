#include "precompiled.h"

#include "ipwidget.h"

// IPWidgetBase
IPWidgetBase::IPWidgetBase( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}

void IPWidgetBase::initialize()
{
    m_layout = new QGridLayout( this );
}

// CameraIPWidget
CameraIPWidget::CameraIPWidget( QWidget* parent )
    : IPWidgetBase( parent )
{
    initialize();
}

void CameraIPWidget::initialize()
{
    m_layout->addWidget( new QLabel( tr( "Camera ip:" ), this ), 0, 0 );

    m_ipLine = new QLineEdit( this );

    m_layout->addWidget( m_ipLine, 0, 1 );

}

QString CameraIPWidget::ip() const
{
    return m_ipLine->text();
}

// StereoIPWidget
StereoIPWidget::StereoIPWidget( QWidget* parent )
    : IPWidgetBase( parent )
{
    initialize();
}

void StereoIPWidget::initialize()
{
    m_layout->addWidget( new QLabel( tr( "Left camera ip:" ), this ), 0, 0 );
    m_leftIpLine = new QLineEdit( this );
    m_layout->addWidget( m_leftIpLine, 0, 1 );

    m_layout->addWidget( new QLabel( tr( "Right camera ip:" ), this ), 1, 0 );
    m_rightIpLine = new QLineEdit( this );
    m_layout->addWidget( m_rightIpLine, 1, 1 );

}

QString StereoIPWidget::leftIp() const
{
    return m_leftIpLine->text();
}

QString StereoIPWidget::rightIp() const
{
    return m_rightIpLine->text();
}

// CameraIPDialog
CameraIPDialog::CameraIPDialog( QWidget* parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void CameraIPDialog::initialize()
{
    setWidget( new CameraIPWidget( this ) );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &DialogBase::accept );

}

CameraIPWidget *CameraIPDialog::widget() const
{
    return dynamic_cast< CameraIPWidget * >( m_widget.data() );
}

QString CameraIPDialog::ip() const
{
    return widget()->ip();
}

// StereoIPDialog
StereoIPDialog::StereoIPDialog( QWidget* parent )
    : DialogBase( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, parent )
{
    initialize();
}

void StereoIPDialog::initialize()
{
    setWidget( new StereoIPWidget( this ) );

    connect( m_buttons, &QDialogButtonBox::accepted, this, &DialogBase::accept );
}

StereoIPWidget *StereoIPDialog::widget() const
{
    return dynamic_cast< StereoIPWidget * >( m_widget.data() );
}

QString StereoIPDialog::leftIp() const
{
    return widget()->leftIp();
}

QString StereoIPDialog::rightIp() const
{
    return widget()->rightIp();
}
