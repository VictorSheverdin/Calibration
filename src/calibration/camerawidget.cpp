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
    m_type = type;
}

TypeComboBox::Type CameraWidgetBase::type() const
{
    return m_type;
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

    m_previewThread.start();

    connect( &m_camera, &MasterCamera::receivedFrame, this, &MonocularCameraWidget::reciveFrame );
    connect( &m_previewThread, &MonocularProcessorThread::updateSignal, this, &MonocularCameraWidget::updateView );

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
    CvImage ret;

    m_updateMutex.lock();
    ret = m_previewWidget->sourceImage();
    m_updateMutex.unlock();

    return ret;
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

void MonocularCameraWidget::reciveFrame()
{
    auto frame = m_camera.getFrame();

    if ( !frame.empty() ) {
        if ( m_type == TypeComboBox::CHECKERBOARD || m_type == TypeComboBox::CIRCLES || m_type == TypeComboBox::ASYM_CIRCLES )
            m_previewThread.processFrame( frame, MonocularProcessorThread::TEMPLATE );
        else if ( m_type == TypeComboBox::ARUCO_MARKERS )
            m_previewThread.processFrame( frame, MonocularProcessorThread::MARKER );

    }

}

void MonocularCameraWidget::updateView()
{
    if ( m_updateMutex.tryLock() ) {

        auto result = m_previewThread.result();

        setSourceImage( result.sourceFrame );
        setPreviewImage( result.preview );
        setTemplateExist( result.exist );

        m_updateMutex.unlock();

    }

}

const TemplateProcessor &MonocularCameraWidget::templateProcessor() const
{
    return m_previewThread.templateProcessor();
}

TemplateProcessor &MonocularCameraWidget::templateProcessor()
{
    return m_previewThread.templateProcessor();
}

const ArucoProcessor &MonocularCameraWidget::markerProcessor() const
{
    return m_previewThread.markerProcessor();
}

ArucoProcessor &MonocularCameraWidget::markerProcessor()
{
    return m_previewThread.markerProcessor();
}

void MonocularCameraWidget::setType( const TypeComboBox::Type type )
{
    CameraWidgetBase::setType( type );

    if ( type == TypeComboBox::CHECKERBOARD )
        templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
    else if ( type == TypeComboBox::CIRCLES )
        templateProcessor().setType( TemplateProcessor::CIRCLES );
    else if ( type == TypeComboBox::ASYM_CIRCLES )
        templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );
}

void MonocularCameraWidget::setCount( const cv::Size &count )
{
    templateProcessor().setCount( count );
}

void MonocularCameraWidget::setTemplateSize( const double value )
{
    templateProcessor().setSize( value );
}

void MonocularCameraWidget::setIntervalSize( const double value )
{
    markerProcessor().setInterval( value );
}

void MonocularCameraWidget::setResizeFlag( const bool value )
{
    templateProcessor().setResizeFlag( value );
    markerProcessor().setResizeFlag( value );
}

void MonocularCameraWidget::setFrameMaximumSize( const unsigned int value )
{
    templateProcessor().setFrameMaximumSize( value );
    markerProcessor().setFrameMaximumSize( value );
}

void MonocularCameraWidget::setAdaptiveThreshold( const bool value )
{
    templateProcessor().setAdaptiveThreshold( value );
}

void MonocularCameraWidget::setNormalizeImage( const bool value )
{
    templateProcessor().setNormalizeImage( value );
}

void MonocularCameraWidget::setFilterQuads( const bool value )
{
    templateProcessor().setFilterQuads( value );
}

void MonocularCameraWidget::setFastCheck( const bool value )
{
    templateProcessor().setFastCheck( value );
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

    m_previewThread.start();

    connect( &m_camera, &StereoCamera::receivedFrame, this, &StereoCameraWidget::reciveFrame );
    connect( &m_previewThread, &StereoProcessorThread::updateSignal, this, &StereoCameraWidget::updateView );

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

StereoFrame StereoCameraWidget::sourceFrame()
{
    StereoFrame ret;

    m_updateMutex.lock();
    ret = m_sourceFrame;
    m_updateMutex.unlock();

    return ret;
}

void StereoCameraWidget::reciveFrame()
{
    auto frame = m_camera.getFrame();

    if ( !frame.empty() ) {
        if ( m_type == TypeComboBox::CHECKERBOARD || m_type == TypeComboBox::CIRCLES || m_type == TypeComboBox::ASYM_CIRCLES )
            m_previewThread.processFrame( frame, StereoProcessorThread::TEMPLATE );
        else if ( m_type == TypeComboBox::ARUCO_MARKERS )
            m_previewThread.processFrame( frame, StereoProcessorThread::MARKER );

    }

}

void StereoCameraWidget::updateView()
{
    if ( m_updateMutex.tryLock() ) {

        auto result = m_previewThread.result();

        m_sourceFrame = result.sourceFrame;

        m_leftCameraWidget->setSourceImage( result.sourceFrame.leftFrame() );
        m_rightCameraWidget->setSourceImage( result.sourceFrame.rightFrame() );

        m_leftCameraWidget->setPreviewImage( result.leftPreview );
        m_leftCameraWidget->setTemplateExist( result.leftExist );

        m_rightCameraWidget->setPreviewImage( result.rightPreview );
        m_rightCameraWidget->setTemplateExist( result.rightExist );

        m_updateMutex.unlock();

    }

}

const TemplateProcessor &StereoCameraWidget::templateProcessor() const
{
    return m_previewThread.templateProcessor();
}

TemplateProcessor &StereoCameraWidget::templateProcessor()
{
    return m_previewThread.templateProcessor();
}

const ArucoProcessor &StereoCameraWidget::markerProcessor() const
{
    return m_previewThread.markerProcessor();
}

ArucoProcessor &StereoCameraWidget::markerProcessor()
{
    return m_previewThread.markerProcessor();
}

void StereoCameraWidget::setType( const TypeComboBox::Type type )
{
    CameraWidgetBase::setType( type );

    if ( type == TypeComboBox::CHECKERBOARD )
        templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
    else if ( type == TypeComboBox::CIRCLES )
        templateProcessor().setType( TemplateProcessor::CIRCLES );
    else if ( type == TypeComboBox::ASYM_CIRCLES )
        templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );
}

void StereoCameraWidget::setCount( const cv::Size &count )
{
    templateProcessor().setCount( count );
}

void StereoCameraWidget::setTemplateSize( const double value )
{
    templateProcessor().setSize( value );
}

void StereoCameraWidget::setIntervalSize( const double value )
{
    markerProcessor().setInterval( value );
}

void StereoCameraWidget::setResizeFlag( const bool value )
{
    templateProcessor().setResizeFlag( value );
    markerProcessor().setResizeFlag( value );
}

void StereoCameraWidget::setFrameMaximumSize( const unsigned int value )
{
    templateProcessor().setFrameMaximumSize( value );
    markerProcessor().setFrameMaximumSize( value );
}

void StereoCameraWidget::setAdaptiveThreshold( const bool value )
{
    templateProcessor().setAdaptiveThreshold( value );
}

void StereoCameraWidget::setNormalizeImage( const bool value )
{
    templateProcessor().setNormalizeImage( value );
}

void StereoCameraWidget::setFilterQuads( const bool value )
{
    templateProcessor().setFilterQuads( value );
}

void StereoCameraWidget::setFastCheck( const bool value )
{
    templateProcessor().setFastCheck( value );
}
