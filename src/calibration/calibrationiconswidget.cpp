#include "src/common/precompiled.h"

#include "calibrationiconswidget.h"

#include "src/common/defs.h"
#include "src/common/functions.h"

// CalibrationIconBase
CalibrationIconBase::CalibrationIconBase( const CvImage image, const cv::Size &frameSize, const std::vector<cv::Point3f> &worldPoints, const QString &text )
    : IconBase( image, text )
{
    setFrameSize( frameSize );
    setWorldPoints( worldPoints );

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

void CalibrationIconBase::setWorldPoints( const std::vector< cv::Point3f > &points )
{
    m_worldPoints = points;
}

const std::vector< cv::Point3f > &CalibrationIconBase::worldPoints() const
{
    return m_worldPoints;
}

// MonocularIcon
MonocularIcon::MonocularIcon( const CvImage previewImage,
                              const cv::Size &frameSize,
                              const std::vector<cv::Point2f> &imagePoints,
                              const std::vector<cv::Point3f> &worldPoints,
                              const QString &text )
    : CalibrationIconBase( previewImage, frameSize, worldPoints, text )
{
    setImagePoints( imagePoints );

    initialize();
}

void MonocularIcon::initialize()
{
}

void MonocularIcon::setImagePoints( const std::vector< cv::Point2f > &points )
{
    m_imagePoints = points;
}

const std::vector< cv::Point2f > &MonocularIcon::imagePoints() const
{
    return m_imagePoints;
}

// StereoIcon
StereoIcon::StereoIcon( const CvImage leftPreviewImage,
                        const CvImage rightPreviewImage,
                        const cv::Size &frameSize,
                        const std::vector< cv::Point2f > &leftImagePoints,
                        const std::vector< cv::Point2f > &rightImagePoints,
                        const std::vector<cv::Point3f> &worldPoints,
                        const QString &text )
    : CalibrationIconBase( makeOverlappedPreview( leftPreviewImage, rightPreviewImage ), frameSize, worldPoints, text )
{
    setLeftPreview( leftPreviewImage );
    setRightPreview( rightPreviewImage );

    setLeftImagePoints( leftImagePoints );
    setRightImagePoints( rightImagePoints );

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

const CvImage StereoIcon::stackedPreview() const
{
    return makeStraightPreview( m_leftPreview, m_rightPreview );
}

void StereoIcon::setLeftImagePoints( const std::vector< cv::Point2f > &points )
{
    m_leftImagePoints = points;
}

std::vector< cv::Point2f > StereoIcon::leftImagePoints() const
{
    return m_leftImagePoints;
}

void StereoIcon::setRightImagePoints( const std::vector< cv::Point2f > &points )
{
    m_rightImagePoints = points;
}

std::vector< cv::Point2f > StereoIcon::rightImagePoints() const
{
    return m_rightImagePoints;
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

