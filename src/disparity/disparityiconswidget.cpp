#include "src/common/precompiled.h"

#include "disparityiconswidget.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

// StereoIcon
DisparityIcon::DisparityIcon( const CvImage &preivewImage, const QString &leftFileName, const QString &rightFileName, const QString &text )
    : IconBase( resizeTo( preivewImage, IconsListWidget::m_iconSize.width() ), text )
{
    setLeftFileName( leftFileName );
    setRightFileName( rightFileName );

    initialize();
}

void DisparityIcon::initialize()
{
    m_time = std::chrono::system_clock::now();
}

void DisparityIcon::setLeftFileName( const QString &fileName )
{
    m_leftFileName = fileName;
}

void DisparityIcon::setRightFileName(const QString &fileName )
{
    m_rightFileName = fileName;
}

const QString &DisparityIcon::leftFileName() const
{
    return m_leftFileName;
}

const QString &DisparityIcon::rightFileName() const
{
    return m_rightFileName;
}

CvImage DisparityIcon::loadLeftImage() const
{
    return CvImage( m_leftFileName.toStdString() );
}

CvImage DisparityIcon::loadRightImage() const
{
    return CvImage( m_rightFileName.toStdString() );
}

StampedStereoImage DisparityIcon::stereoFrame() const
{
    return StampedStereoImage( StampedImage( m_time, loadLeftImage() ), StampedImage( m_time, loadRightImage() ) );
}

void DisparityIcon::setTime( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    m_time = time;
}

const std::chrono::time_point< std::chrono::system_clock > &DisparityIcon::time() const
{
    return m_time;
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
