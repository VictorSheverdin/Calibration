#include "src/common/precompiled.h"

#include "calibrationiconswidget.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

// CalibrationIconBase
CalibrationIconBase::CalibrationIconBase( const CvImage image, const int number )
    : IconBase( image, number )
{
    initialize();
}

void CalibrationIconBase::initialize()
{
}

bool CalibrationIconBase::isMonocularIcon() const
{
    return toMonocularIcon();
}

bool CalibrationIconBase::isStereoIcon() const
{
    return toStereoIcon();
}

MonocularIcon *CalibrationIconBase::toMonocularIcon()
{
    return dynamic_cast< MonocularIcon * >( this );
}

StereoIcon *CalibrationIconBase::toStereoIcon()
{
    return dynamic_cast< StereoIcon * >( this );
}

const MonocularIcon *CalibrationIconBase::toMonocularIcon() const
{
    return dynamic_cast< const MonocularIcon * >( this );
}

const StereoIcon *CalibrationIconBase::toStereoIcon() const
{
    return dynamic_cast< const StereoIcon * >( this );
}


// MonocularIcon
MonocularIcon::MonocularIcon( const CvImage previewImage, const CvImage sourceImage, const int number )
    : CalibrationIconBase( previewImage, number )
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
    : CalibrationIconBase( makeOverlappedPreview( leftPreviewImage, rightPreviewImage ), number )
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

// IconsListWidget
CalibrationIconsWidget::CalibrationIconsWidget( QWidget *parent )
    : SuperClass( parent )
{
    initialize();
}

void CalibrationIconsWidget::initialize()
{
    setWrapping( true );

    connect( this, &CalibrationIconsWidget::itemDoubleClicked,
                [&]( QListWidgetItem *item ) {
                    auto itemCast = dynamic_cast< CalibrationIconBase * >( item );

                    if ( itemCast )
                        emit iconActivated( itemCast );

                }

    );

}

void CalibrationIconsWidget::addIcon( CalibrationIconBase *icon )
{
    SuperClass::addIcon( icon );
}

void CalibrationIconsWidget::insertIcon( CalibrationIconBase *icon )
{
    SuperClass::insertIcon( icon );
}

QList< CalibrationIconBase* > CalibrationIconsWidget::icons() const
{
    QList< CalibrationIconBase* > ret;

    auto list = SuperClass::icons();

    for ( auto &i : list ) {
        auto itemCast = dynamic_cast< CalibrationIconBase* >( i );
        if ( itemCast )
            ret.push_back( itemCast );
    }

    return ret;

}

