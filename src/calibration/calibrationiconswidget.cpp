#include "src/common/precompiled.h"

#include "calibrationiconswidget.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

// CalibrationIconBase
CalibrationIconBase::CalibrationIconBase(const CvImage image, const cv::Size &frameSize, const QString &text )
    : IconBase( image, text )
{
    setFrameSize( frameSize );

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

void CalibrationIconBase::setFrameSize( const cv::Size &size )
{
    m_frameSize = size;
}

const cv::Size &CalibrationIconBase::frameSize() const
{
    return m_frameSize;
}

// MonocularIcon
MonocularIcon::MonocularIcon(const CvImage previewImage, const cv::Size &frameSize, const std::vector<cv::Point2f> &points, const QString &text )
    : CalibrationIconBase( previewImage, frameSize, text )
{
    setPoints( points );

    initialize();
}

void MonocularIcon::initialize()
{
}

void MonocularIcon::setPoints( const std::vector< cv::Point2f > &points )
{
    m_points = points;
}

const std::vector<cv::Point2f> &MonocularIcon::points() const
{
    return m_points;
}

// StereoIcon
StereoIcon::StereoIcon(const CvImage leftPreviewImage, const CvImage rightPreviewImage, const cv::Size &frameSize,
                       const std::vector< cv::Point2f > &leftPoints, const std::vector< cv::Point2f > &rightPoints, const QString &text )
    : CalibrationIconBase( makeOverlappedPreview( leftPreviewImage, rightPreviewImage ), frameSize, text )
{
    setLeftPreview( leftPreviewImage );
    setRightPreview( leftPreviewImage );

    setLeftPoints( leftPoints );
    setRightPoints( rightPoints );

    initialize();
}

void StereoIcon::initialize()
{
}

void StereoIcon::setLeftPreview( const CvImage &image )
{
    m_leftPreview = image;
}

void StereoIcon::setRightPreview(const CvImage &image)
{
    m_rightPreview = image;
}

const CvImage &StereoIcon::leftPreview() const
{
    return m_leftPreview;
}

const CvImage &StereoIcon::rightPreview() const
{
    return m_rightPreview;
}

const CvImage StereoIcon::straightPreview() const
{
    return makeStraightPreview( m_leftPreview, m_rightPreview );
}

void StereoIcon::setLeftPoints( const std::vector< cv::Point2f > &points )
{
    m_leftPoints = points;
}

std::vector< cv::Point2f > StereoIcon::leftPoints() const
{
    return m_leftPoints;
}

void StereoIcon::setRightPoints( const std::vector< cv::Point2f > &points )
{
    m_rightPoints = points;
}

std::vector< cv::Point2f > StereoIcon::rightPoints() const
{
    return m_rightPoints;
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

