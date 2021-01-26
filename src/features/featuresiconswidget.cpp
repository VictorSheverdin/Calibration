#include "src/common/precompiled.h"

#include "featuresiconswidget.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

// StereoIcon
FeaturesIcon::FeaturesIcon( const CvImage &preivewImage, const QString &leftFileName, const QString &rightFileName, const QString &text )
    : IconBase( resizeTo( preivewImage, IconsListWidget::m_iconSize.width() ), text )
{
    setLeftFileName( leftFileName );
    setRightFileName( rightFileName );

    initialize();
}

void FeaturesIcon::initialize()
{
    setTime( std::chrono::system_clock::now() );
}

void FeaturesIcon::setLeftFileName( const QString &fileName )
{
    m_fileName1 = fileName;
}

void FeaturesIcon::setRightFileName(const QString &fileName )
{
    m_fileName2 = fileName;
}

const QString &FeaturesIcon::leftFileName() const
{
    return m_fileName1;
}

const QString &FeaturesIcon::rightFileName() const
{
    return m_fileName2;
}

CvImage FeaturesIcon::loadLeftImage() const
{
    return CvImage( m_fileName1.toStdString() );
}

CvImage FeaturesIcon::loadRightImage() const
{
    return CvImage( m_fileName2.toStdString() );
}

void FeaturesIcon::setTime( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    m_time = time;
}

const std::chrono::time_point< std::chrono::system_clock > &FeaturesIcon::time() const
{
    return m_time;
}

// IconsListWidget
FeaturesIconsWidget::FeaturesIconsWidget( QWidget *parent )
    : SuperClass( parent )
{
    initialize();
}

void FeaturesIconsWidget::initialize()
{
    setWrapping( false );

    connect( this, &FeaturesIconsWidget::itemDoubleClicked,
                [&]( QListWidgetItem *item ) {
                    auto itemCast = dynamic_cast< FeaturesIcon * >( item );

                    if ( itemCast )
                        emit iconActivated( itemCast );

                }

    );

}

void FeaturesIconsWidget::addIcon( FeaturesIcon *icon )
{
    SuperClass::addIcon( icon );
}

void FeaturesIconsWidget::insertIcon( FeaturesIcon *icon )
{
    SuperClass::insertIcon( icon );
}

QList< FeaturesIcon * > FeaturesIconsWidget::icons() const
{
    QList< FeaturesIcon* > ret;

    auto list = SuperClass::icons();

    for ( auto &i : list ) {
        auto itemCast = dynamic_cast< FeaturesIcon* >( i );
        if ( itemCast )
            ret.push_back( itemCast );
    }

    return ret;

}

FeaturesIcon *FeaturesIconsWidget::currentIcon() const
{
    return dynamic_cast< FeaturesIcon * >( currentItem() );
}
