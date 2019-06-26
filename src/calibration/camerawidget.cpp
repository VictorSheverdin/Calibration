#include "src/common/precompiled.h"

#include "camerawidget.h"

#include "previewwidget.h"

#include "src/common/functions.h"

// CameraWidgetBase
CameraWidgetBase::CameraWidgetBase(QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void CameraWidgetBase::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
}

void CameraWidgetBase::setType( const TemplateProcessor::Type type )
{
    m_previewProcessor.setType( type );
}

void CameraWidgetBase::setCount( const cv::Size &count )
{
    m_previewProcessor.setCount( count );
}

void CameraWidgetBase::setTemplateSize( const double value )
{
    m_previewProcessor.setSize( value );
}

void CameraWidgetBase::setResizeFlag( const bool value )
{
    m_previewProcessor.setResizeFlag( value );
}

void CameraWidgetBase::setFrameMaximumSize( const unsigned int value )
{
    m_previewProcessor.setFrameMaximumSize( value );
}

void CameraWidgetBase::setAdaptiveThreshold( const bool value )
{
    m_previewProcessor.setAdaptiveThreshold( value );
}

void CameraWidgetBase::setNormalizeImage( const bool value )
{
    m_previewProcessor.setNormalizeImage( value );
}

void CameraWidgetBase::setFilterQuads( const bool value )
{
    m_previewProcessor.setFilterQuads( value );
}

void CameraWidgetBase::setFastCheck( const bool value )
{
    m_previewProcessor.setFastCheck( value );
}

TemplateProcessor::Type CameraWidgetBase::templateType() const
{
    return m_previewProcessor.type();
}

const cv::Size &CameraWidgetBase::templateCount() const
{
    return m_previewProcessor.count();
}

double CameraWidgetBase::templateSize() const
{
    return m_previewProcessor.size();
}

bool CameraWidgetBase::resizeFlag() const
{
    return m_previewProcessor.resizeFlag();
}

unsigned int CameraWidgetBase::frameMaximumFlag() const
{
    return m_previewProcessor.frameMaximumFlag();
}

bool CameraWidgetBase::adaptiveThreshold() const
{
    return m_previewProcessor.adaptiveThreshold();
}

bool CameraWidgetBase::normalizeImage() const
{
    return m_previewProcessor.normalizeImage();
}

bool CameraWidgetBase::filterQuads() const
{
    return m_previewProcessor.filterQuads();
}

bool CameraWidgetBase::fastCheck() const
{
    return m_previewProcessor.fastCheck();
}

// MonocularCameraWidget
MonocularCameraWidget::MonocularCameraWidget( const QString &cameraIp, QWidget* parent )
    : CameraWidgetBase( parent ), m_camera( cameraIp.toStdString(), parent )
{
    initialize();
}

void MonocularCameraWidget::initialize()
{
    m_previewWidget = new CameraPreviewWidget( this );
    addWidget( m_previewWidget );

    connect( &m_camera, &MasterCamera::receivedFrame, this, &MonocularCameraWidget::updateFrame );

    startTimer( 1000 / 30 );

}

void MonocularCameraWidget::setSourceImage(const CvImage image)
{
    m_previewWidget->setSourceImage( image );
}

void MonocularCameraWidget::setPreviewImage(const CvImage image)
{
    m_previewWidget->setPreviewImage( image );
}

void MonocularCameraWidget::setPreviewPoints( const std::vector< cv::Point2f > &points )
{
    m_previewWidget->setPreviewPoints( points );
}

const CvImage MonocularCameraWidget::sourceImage() const
{
    return m_previewWidget->sourceImage();
}

const CvImage MonocularCameraWidget::previewImage() const
{
    return m_previewWidget->previewImage();
}

const std::vector<cv::Point2f> &MonocularCameraWidget::previewPoints() const
{
    return m_previewWidget->previewPoints();
}

bool MonocularCameraWidget::isTemplateExist() const
{
    return m_previewWidget->isTemplateExist();
}

void MonocularCameraWidget::updateFrame()
{
    auto frame = m_camera.getFrame();

    if ( !frame.empty() ) {
        setSourceImage( frame );

        CvImage procFrame;
        std::vector< cv::Point2f > previewPoints;

        m_previewProcessor.processPreview( frame, &procFrame, &previewPoints );

        setPreviewImage( procFrame );
        setPreviewPoints( previewPoints );

    }

}

// StereoCameraWidget
StereoCameraWidget::StereoCameraWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget* parent )
    : CameraWidgetBase( parent ), m_camera( leftCameraIp.toStdString(), rightCameraIp.toStdString(), parent )
{
    initialize();
}

void StereoCameraWidget::initialize()
{
    m_leftCameraWidget = new CameraPreviewWidget( this );
    m_rightCameraWidget = new CameraPreviewWidget( this );

    addWidget( m_leftCameraWidget );
    addWidget( m_rightCameraWidget );

    connect( &m_camera, &StereoCamera::receivedFrame, this, &StereoCameraWidget::updateFrame );

    startTimer( 1000 / 30 );

}

const CvImage StereoCameraWidget::leftSourceImage() const
{
    return m_leftCameraWidget->sourceImage();
}

const CvImage StereoCameraWidget::leftDisplayedImage() const
{
    return m_leftCameraWidget->previewImage();
}

const CvImage StereoCameraWidget::rightSourceImage() const
{
    return m_rightCameraWidget->sourceImage();
}

const CvImage StereoCameraWidget::rightDisplayedImage() const
{
    return m_rightCameraWidget->previewImage();
}

void StereoCameraWidget::setLeftSourceImage( const CvImage image )
{
    return m_leftCameraWidget->setSourceImage( image );
}

void StereoCameraWidget::setLeftDisplayedImage( const CvImage image )
{
    return m_leftCameraWidget->setPreviewImage( image );
}

void StereoCameraWidget::setLeftPreviewPoints( const std::vector< cv::Point2f > &points )
{
    m_leftCameraWidget->setPreviewPoints( points );
}

void StereoCameraWidget::setRightSourceImage( const CvImage image )
{
    return m_rightCameraWidget->setSourceImage( image );
}

void StereoCameraWidget::setRightDisplayedImage( const CvImage image )
{
    return m_rightCameraWidget->setPreviewImage( image );
}

void StereoCameraWidget::setRightPreviewPoints(const std::vector<cv::Point2f> &points )
{
    m_rightCameraWidget->setPreviewPoints( points );
}

bool StereoCameraWidget::isTemplateExist() const
{
    return m_leftCameraWidget->isTemplateExist() || m_rightCameraWidget->isTemplateExist();
}

const StereoImage &StereoCameraWidget::stereoImage() const
{
    return m_camera.getFrame();
}

void StereoCameraWidget::updateFrame()
{
    if ( m_updateMutex.tryLock() ) {

        auto frame = m_camera.getFrame();

        if ( !frame.empty() ) {

            m_leftCameraWidget->setSourceImage( frame.leftImage() );
            m_rightCameraWidget->setSourceImage( frame.rightImage() );

            CvImage leftProcFrame;
            std::vector< cv::Point2f > leftPreviewPoints;

            CvImage rightProcFrame;
            std::vector< cv::Point2f > rightPreviewPoints;

            m_previewProcessor.processPreview( frame.leftImage(), &leftProcFrame, &leftPreviewPoints );
            m_previewProcessor.processPreview( frame.rightImage(), &rightProcFrame, &rightPreviewPoints );

            m_leftCameraWidget->setPreviewImage( leftProcFrame );
            m_leftCameraWidget->setPreviewPoints( leftPreviewPoints );

            m_rightCameraWidget->setPreviewImage( rightProcFrame );
            m_rightCameraWidget->setPreviewPoints( rightPreviewPoints );
        }

        m_updateMutex.unlock();

    }

}

// TripleCameraWidget
TripleCameraWidget::TripleCameraWidget( const int camera1Index, const int camera2Index, const int camera3Index, QWidget* parent )
    : CameraWidgetBase( parent )
{
    initialize( camera1Index, camera2Index, camera3Index );

}

void TripleCameraWidget::initialize( const int camera1Index, const int camera2Index, const int camera3Index )
{
    m_videoCaptures[0].open( camera1Index );
    m_videoCaptures[1].open( camera2Index );
    m_videoCaptures[2].open( camera3Index );

    m_cameraWidgets[0] = new CameraPreviewWidget( this );
    m_cameraWidgets[1] = new CameraPreviewWidget( this );
    m_cameraWidgets[2] = new CameraPreviewWidget( this );

    addWidget( m_cameraWidgets[0] );
    addWidget( m_cameraWidgets[1] );
    addWidget( m_cameraWidgets[2] );

}

const CvImage TripleCameraWidget::sourceImage( const unsigned int cameraIndex ) const
{
    if( cameraIndex >= 0 && cameraIndex < 3 )
        return m_cameraWidgets[ cameraIndex ]->sourceImage();
    else
        return CvImage();

}

const CvImage TripleCameraWidget::previewImage( const unsigned int cameraIndex ) const
{
    if( cameraIndex >= 0 && cameraIndex < 3 )
        return m_cameraWidgets[ cameraIndex ]->previewImage();
    else
        return CvImage();

}

CvImage TripleCameraWidget::createPreview( const CvImage &previewImage1, const CvImage &previewImage2, const CvImage &previewImage3 )
{
/*    CvImage result( std::max( leftPreviewImage.height(), rightPreviewImage.height() ),
                    leftPreviewImage.width() / 2 + rightPreviewImage.width(),
                    leftPreviewImage.type(), cv::Scalar( 0, 0, 0, 0) );

    leftPreviewImage.copyTo( result( cv::Rect( 0, 0, leftPreviewImage.width(), leftPreviewImage.height() ) ) );
    rightPreviewImage.copyTo( result( cv::Rect( result.width() - rightPreviewImage.width(), 0, rightPreviewImage.width(), rightPreviewImage.height() ) ) );

    return result;*/
}

void TripleCameraWidget::setSource1Image( const unsigned int cameraIndex, const CvImage image )
{
    if( cameraIndex >= 0 && cameraIndex < 3 )
        m_cameraWidgets[ cameraIndex ]->setSourceImage( image );

}

void TripleCameraWidget::setPreviewImage( const unsigned int cameraIndex, const CvImage image )
{
    if( cameraIndex >= 0 && cameraIndex < 3 )
        m_cameraWidgets[ cameraIndex ]->setSourceImage( image );

}

void TripleCameraWidget::updatePreview()
{
    for ( int i = 0; i < 3; ++i )
        updatePreview( i );
}

void TripleCameraWidget::updatePreview( const unsigned int cameraIndex )
{

}

