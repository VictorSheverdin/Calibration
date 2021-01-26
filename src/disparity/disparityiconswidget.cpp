#include "src/common/precompiled.h"

#include "disparityiconswidget.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

// DisparityIconBase
DisparityIconBase::DisparityIconBase( const CvImage image, const QString &text )
    : IconBase( image, text )
{
}

// StereoIcon
DisparityIcon::DisparityIcon( const CvImage &preivewImage, const QString &leftFileName, const QString &rightFileName, const QString &text )
    : DisparityIconBase( resizeTo( preivewImage, IconsListWidget::m_iconSize.width() ), text )
{
    setLeftFileName( leftFileName );
    setRightFileName( rightFileName );

    initialize();
}

void DisparityIcon::initialize()
{
    setTime( std::chrono::system_clock::now() );
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

// DisparityResultIcon
DisparityResultIcon::DisparityResultIcon( const CvImage &preivewImage, const QString &colorFileName, const QString &disparityFileName, const QString &text )
    : DisparityIconBase( resizeTo( preivewImage, IconsListWidget::m_iconSize.width() ), text )
{
    setColorFileName( colorFileName );
    setDisparityFileName( disparityFileName );

    initialize();
}

void DisparityResultIcon::initialize()
{
    setTime( std::chrono::system_clock::now() );
}

void DisparityResultIcon::setColorFileName( const QString &fileName )
{
    m_colorFileName = fileName;
}

void DisparityResultIcon::setDisparityFileName( const QString &fileName )
{
    m_disparityFileName = fileName;
}

const QString &DisparityResultIcon::colorFileName() const
{
    return m_colorFileName;
}

const QString &DisparityResultIcon::disparityFileName() const
{
    return m_disparityFileName;
}

CvImage DisparityResultIcon::colorImage() const
{
    return CvImage( m_colorFileName.toStdString() );
}

CvImage DisparityResultIcon::disparityImage() const
{
    return CvImage( m_disparityFileName.toStdString() );
}

void DisparityResultIcon::setTime( const std::chrono::time_point< std::chrono::system_clock > &time )
{
    m_time = time;
}

const std::chrono::time_point< std::chrono::system_clock > &DisparityResultIcon::time() const
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
                    auto itemCast = dynamic_cast< DisparityIconBase * >( item );

                    if ( itemCast )
                        emit iconActivated( itemCast );

                }

    );

}

void DisparityIconsWidget::addIcon( DisparityIconBase *icon )
{
    SuperClass::addIcon( icon );
}

void DisparityIconsWidget::insertIcon( DisparityIconBase *icon )
{
    SuperClass::insertIcon( icon );
}

QList< DisparityIconBase * > DisparityIconsWidget::icons() const
{
    QList< DisparityIconBase* > ret;

    auto list = SuperClass::icons();

    for ( auto &i : list ) {
        auto itemCast = dynamic_cast< DisparityIconBase* >( i );
        if ( itemCast )
            ret.push_back( itemCast );
    }

    return ret;

}

DisparityIconBase *DisparityIconsWidget::currentIcon() const
{
    return dynamic_cast< DisparityIconBase * >( currentItem() );
}
