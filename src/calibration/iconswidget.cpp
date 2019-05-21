#include "src/common/precompiled.h"

#include "iconswidget.h"
#include "src/common/defs.h"
#include "src/common/functions.h"

// IconBase
IconBase::IconBase( const CvImage image, const int number )
    : QListWidgetItem( QPixmap::fromImage( QtImage( image ) ) , QObject::tr("Frame") + " " + QString::number( number ) ), m_previewImage( image )
{
    initialize();
}

void IconBase::initialize()
{
}

bool IconBase::isMonocularIcon() const
{
    return toMonocularIcon();
}

bool IconBase::isStereoIcon() const
{
    return toStereoIcon();
}

MonocularIcon *IconBase::toMonocularIcon()
{
    return dynamic_cast< MonocularIcon * >( this );
}

StereoIcon *IconBase::toStereoIcon()
{
    return dynamic_cast< StereoIcon * >( this );
}

const MonocularIcon *IconBase::toMonocularIcon() const
{
    return dynamic_cast< const MonocularIcon * >( this );
}

const StereoIcon *IconBase::toStereoIcon() const
{
    return dynamic_cast< const StereoIcon * >( this );
}

const CvImage &IconBase::previewImage() const
{
    return m_previewImage;
}

// MonocularIcon
MonocularIcon::MonocularIcon( const CvImage previewImage, const CvImage sourceImage, const int number )
    : IconBase( previewImage, number )
{
    setSourceImage( sourceImage );

    initialize();
}

void MonocularIcon::initialize()
{
}

void MonocularIcon::setSourceImage( const CvImage &image )
{
    m_sourceImage = image;
}

const CvImage MonocularIcon::sourceImage() const
{
    return m_sourceImage;
}

void MonocularIcon::setPreviewPoints( std::vector< cv::Point2f > &points )
{
    m_previewPoints = points;
}

std::vector< cv::Point2f > MonocularIcon::previewPoints() const
{
    return m_previewPoints;
}

// StereoIcon
StereoIcon::StereoIcon( const CvImage leftPreviewImage, const CvImage rightPreviewImage, const CvImage leftSourceImage, const CvImage rightSourceImage, const int number )
    : IconBase( makeOverlappedPreview( leftPreviewImage, rightPreviewImage ), number )
{
    setStraightPreview( makeStraightPreview( leftPreviewImage, rightPreviewImage ) );
    setLeftSourceImage( leftSourceImage );
    setRightSourceImage( rightSourceImage );

    initialize();
}

void StereoIcon::initialize()
{
}

void StereoIcon::setStraightPreview( const CvImage &image )
{
    m_straightPreview = image;
}

void StereoIcon::setLeftSourceImage( const CvImage &image )
{
    m_leftSourceImage = image;
}

void StereoIcon::setRightSourceImage( const CvImage &image )
{
    m_rightSourceImage = image;
}

const CvImage &StereoIcon::straightPreview() const
{
    return m_straightPreview;
}

const CvImage StereoIcon::leftSourceImage() const
{
    return m_leftSourceImage;
}

const CvImage StereoIcon::rightSourceImage() const
{
    return m_rightSourceImage;
}

void StereoIcon::setLeftPreviewPoints( std::vector< cv::Point2f > &points )
{
    m_previewLeftPoints = points;
}

std::vector< cv::Point2f > StereoIcon::leftPreviewPoints() const
{
    return m_previewLeftPoints;
}

void StereoIcon::setRightPreviewPoints( std::vector< cv::Point2f > &points )
{
    m_previewRightPoints = points;
}

std::vector< cv::Point2f > StereoIcon::rightPreviewPoints() const
{
    return m_previewRightPoints;
}

CvImage StereoIcon::makeOverlappedPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage )
{
    return stackImages( leftPreviewImage, rightPreviewImage, 0.5 );
}

CvImage StereoIcon::makeStraightPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage )
{
    return stackImages( leftPreviewImage, rightPreviewImage, 1 );
}

// IconsWidget
const QSize IconsWidget::m_iconSize( 200, 200 );

IconsWidget::IconsWidget( QWidget *parent )
    : SuperClass( parent )
{
    initialize();
}

void IconsWidget::initialize()
{
    setIconSize( m_iconSize );
    setViewMode( IconMode );
    setWrapping( true );

    connect( this, &IconsWidget::itemDoubleClicked,
                [&]( QListWidgetItem *item ) {
                    auto itemCast = dynamic_cast< IconBase * >( item );

                    if ( itemCast )
                        emit iconActivated( itemCast );

                }

    );

}

void IconsWidget::addIcon( IconBase *icon )
{
    addItem( icon );
}

void IconsWidget::insertIcon(IconBase *icon )
{
    insertItem( 0, icon );
}

QList< IconBase* > IconsWidget::icons() const
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


