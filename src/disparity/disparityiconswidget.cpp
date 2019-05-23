#include "src/common/precompiled.h"

#include "disparityiconswidget.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

// StereoIcon
DisparityIcon::DisparityIcon( const CvImage leftSourceImage, const CvImage rightSourceImage, const int number )
    : IconBase( makeOverlappedPreview( leftSourceImage, rightSourceImage ), number )
{
    setLeftImage( leftSourceImage );
    setRightImage( rightSourceImage );

    initialize();
}

void DisparityIcon::initialize()
{
}

void DisparityIcon::setLeftImage( const CvImage &image )
{
    m_leftImage = image;
}

void DisparityIcon::setRightImage( const CvImage &image )
{
    m_rightImage = image;
}

const CvImage &DisparityIcon::leftImage() const
{
    return m_leftImage;
}

const CvImage &DisparityIcon::rightImage() const
{
    return m_rightImage;
}

// IconsListWidget
DisparityIconsWidget::DisparityIconsWidget( QWidget *parent )
    : SuperClass( parent )
{
    initialize();
}

void DisparityIconsWidget::initialize()
{
    setWrapping( false );

    connect( this, &DisparityIconsWidget::itemDoubleClicked,
                [&]( QListWidgetItem *item ) {
                    auto itemCast = dynamic_cast< DisparityIcon * >( item );

                    if ( itemCast )
                        emit iconActivated( itemCast );

                }

    );

}

void DisparityIconsWidget::addIcon( DisparityIcon *icon )
{
    SuperClass::addIcon( icon );
}

void DisparityIconsWidget::insertIcon( DisparityIcon *icon )
{
    SuperClass::insertIcon( icon );
}

QList< DisparityIcon* > DisparityIconsWidget::icons() const
{
    QList< DisparityIcon* > ret;

    auto list = SuperClass::icons();

    for ( auto &i : list ) {
        auto itemCast = dynamic_cast< DisparityIcon* >( i );
        if ( itemCast )
            ret.push_back( itemCast );
    }

    return ret;

}

DisparityIcon *DisparityIconsWidget::currentIcon() const
{
    return dynamic_cast< DisparityIcon * >( currentItem() );
}
