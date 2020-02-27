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

void CameraWidgetBase::setType( const TypeComboBox::Type type )
{
    if ( type == TypeComboBox::CHECKERBOARD )
        m_previewTemplateProcessor.setType( TemplateProcessor::CHECKERBOARD );
    else if ( type == TypeComboBox::CIRCLES )
        m_previewTemplateProcessor.setType( TemplateProcessor::CIRCLES );
    else if ( type == TypeComboBox::ASYM_CIRCLES )
        m_previewTemplateProcessor.setType( TemplateProcessor::ASYM_CIRCLES );

    m_type = type;
}

void CameraWidgetBase::setCount( const cv::Size &count )
{
    m_previewTemplateProcessor.setCount( count );
}

void CameraWidgetBase::setTemplateSize( const double value )
{
    m_previewTemplateProcessor.setSize( value );
}

void CameraWidgetBase::setIntervalSize( const double value )
{
    m_previewArucoProcessor.setInterval( value );
}

void CameraWidgetBase::setResizeFlag( const bool value )
{
    m_previewTemplateProcessor.setResizeFlag( value );
    m_previewArucoProcessor.setResizeFlag( value );
}

void CameraWidgetBase::setFrameMaximumSize( const unsigned int value )
{
    m_previewTemplateProcessor.setFrameMaximumSize( value );
    m_previewArucoProcessor.setFrameMaximumSize( value );
}

void CameraWidgetBase::setAdaptiveThreshold( const bool value )
{
    m_previewTemplateProcessor.setAdaptiveThreshold( value );
}

void CameraWidgetBase::setNormalizeImage( const bool value )
{
    m_previewTemplateProcessor.setNormalizeImage( value );
}

void CameraWidgetBase::setFilterQuads( const bool value )
{
    m_previewTemplateProcessor.setFilterQuads( value );
}

void CameraWidgetBase::setFastCheck( const bool value )
{
    m_previewTemplateProcessor.setFastCheck( value );
}

TypeComboBox::Type CameraWidgetBase::templateType() const
{
    return m_type;
}

const cv::Size &CameraWidgetBase::templateCount() const
{
    return m_previewTemplateProcessor.count();
}

double CameraWidgetBase::templateSize() const
{
    return m_previewTemplateProcessor.size();
}

double CameraWidgetBase::intervalSize() const
{
    return m_previewArucoProcessor.interval();
}

bool CameraWidgetBase::resizeFlag() const
{
    return m_previewTemplateProcessor.resizeFlag();
}

unsigned int CameraWidgetBase::frameMaximumSize() const
{
    return m_previewTemplateProcessor.frameMaximumSize();
}

bool CameraWidgetBase::adaptiveThreshold() const
{
    return m_previewTemplateProcessor.adaptiveThreshold();
}

bool CameraWidgetBase::normalizeImage() const
{
    return m_previewTemplateProcessor.normalizeImage();
}

bool CameraWidgetBase::filterQuads() const
{
    return m_previewTemplateProcessor.filterQuads();
}

bool CameraWidgetBase::fastCheck() const
{
    return m_previewTemplateProcessor.fastCheck();
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

}

void MonocularCameraWidget::setSourceImage(const CvImage image)
{
    m_previewWidget->setSourceImage( image );
}

void MonocularCameraWidget::setPreviewImage(const CvImage image)
{
    m_previewWidget->setPreviewImage( image );
}

const CvImage MonocularCameraWidget::sourceImage() const
{
    return m_previewWidget->sourceImage();
}

const CvImage MonocularCameraWidget::previewImage() const
{
    return m_previewWidget->previewImage();
}

void MonocularCameraWidget::setTemplateExist( const bool value )
{
    m_previewWidget->setTemplateExist( value );
}

bool MonocularCameraWidget::isTemplateExist() const
{
    return m_previewWidget->isTemplateExist();
}

void MonocularCameraWidget::updateFrame()
{
    if ( m_updateMutex.tryLock() ) {

        auto frame = m_camera.getFrame();

        if ( !frame.empty() ) {
            setSourceImage( frame );

            bool exist = false;
            CvImage procFrame;

            if ( m_type == TypeComboBox::CHECKERBOARD || m_type == TypeComboBox::CIRCLES || m_type == TypeComboBox::ASYM_CIRCLES )
                exist = m_previewTemplateProcessor.processFrame( frame, &procFrame );
            else if ( m_type == TypeComboBox::ARUCO_MARKERS )
                exist = m_previewArucoProcessor.processFrame( frame, &procFrame );

            setPreviewImage( procFrame );
            setTemplateExist( exist );

        }

        m_updateMutex.unlock();

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

void StereoCameraWidget::setRightSourceImage( const CvImage image )
{
    return m_rightCameraWidget->setSourceImage( image );
}

void StereoCameraWidget::setRightDisplayedImage( const CvImage image )
{
    return m_rightCameraWidget->setPreviewImage( image );
}

bool StereoCameraWidget::isTemplateExist() const
{
    return m_leftCameraWidget->isTemplateExist() || m_rightCameraWidget->isTemplateExist();
}

StereoFrame StereoCameraWidget::stereoFrame()
{
    return m_camera.getFrame();
}

void StereoCameraWidget::updateFrame()
{
    if ( m_updateMutex.tryLock() ) {

        auto frame = m_camera.getFrame();

        if ( !frame.empty() ) {

            m_leftCameraWidget->setSourceImage( frame.leftFrame() );
            m_rightCameraWidget->setSourceImage( frame.rightFrame() );

            bool leftExist = false;
            bool rightExist = false;
            CvImage leftProcFrame;
            CvImage rightProcFrame;

            if ( m_type == TypeComboBox::CHECKERBOARD || m_type == TypeComboBox::CIRCLES || m_type == TypeComboBox::ASYM_CIRCLES ) {
                leftExist = m_previewTemplateProcessor.processFrame( frame.leftFrame(), &leftProcFrame );
                rightExist = m_previewTemplateProcessor.processFrame( frame.rightFrame(), &rightProcFrame );
            }
            else if ( m_type == TypeComboBox::ARUCO_MARKERS ) {
                leftExist = m_previewArucoProcessor.processFrame( frame.leftFrame(), &leftProcFrame );
                rightExist = m_previewArucoProcessor.processFrame( frame.rightFrame(), &rightProcFrame );
            }

            m_leftCameraWidget->setPreviewImage( leftProcFrame );
            m_leftCameraWidget->setTemplateExist( leftExist );

            m_rightCameraWidget->setPreviewImage( rightProcFrame );
            m_rightCameraWidget->setTemplateExist( rightExist );

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
