#include "precompiled.h"

#include "iconswidget.h"
#include "defs.h"
#include "functions.h"

// IconBase
IconBase::IconBase( const CvImage image, const int number )
    : QListWidgetItem( QPixmap::fromImage( QtImage( image ) ) , QObject::tr("Frame") + " " + QString::number( number ) ), m_previewImage( image )
{
    initialize();
}

void IconBase::initialize()
{
}

const CvImage &IconBase::previewImage() const
{
    return m_previewImage;
}

// IconsListWidget
const QSize IconsListWidget::m_iconSize( 200, 200 );

IconsListWidget::IconsListWidget( QWidget *parent )
    : SuperClass( parent )
{
    initialize();
}

void IconsListWidget::initialize()
{
    setIconSize( m_iconSize );
    setViewMode( IconMode );
}

void IconsListWidget::addIcon( IconBase *icon )
{
    addItem( icon );
}

void IconsListWidget::insertIcon(IconBase *icon )
{
    insertItem( 0, icon );
}

QList< IconBase* > IconsListWidget::icons() const
{
    QList< IconBase* > ret;

    for ( auto i = 0; i < count(); ++i ) {
        auto item = this->item( i );
        if ( item ) {
            auto itemCast = dynamic_cast< IconBase* >( item );
            if ( itemCast )
                ret.push_back( itemCast );
        }

    }

    return ret;

}


