#include "precompiled.h"

#include "camerawidget.h"

#include "previewwidget.h"

// CameraWidgetBase
void checkVimbaStatus(VmbErrorType status, std::string message)
{
    if (status != VmbErrorSuccess)
    {
        throw std::runtime_error(
            message + "; status = " + std::to_string(status));
    }
}

template<typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;
    checkVimbaStatus( camera->GetFeatureByName(key.data(), feature ),
        "Could not access " + key);
    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );
}

CameraWidgetBase::CameraWidgetBase( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void CameraWidgetBase::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );

    m_timerId = startTimer( m_aquireInterval );
}

void CameraWidgetBase::setType( const TemplateProcessor::Type type )
{
    m_processor.setType( type );
}

void CameraWidgetBase::setCount( const cv::Size &count )
{
    m_processor.setCount( count );
}

void CameraWidgetBase::setTemplateSize( const double value )
{
    m_processor.setSize( value );
}

void CameraWidgetBase::setResizeFlag( const bool value )
{
    m_processor.setResizeFlag( value );
}

void CameraWidgetBase::setFrameMaximumSize( const unsigned int value )
{
    m_processor.setFrameMaximumSize( value );
}

void CameraWidgetBase::setAdaptiveThreshold( const bool value )
{
    m_processor.setAdaptiveThreshold( value );
}

void CameraWidgetBase::setNormalizeImage( const bool value )
{
    m_processor.setNormalizeImage( value );
}

void CameraWidgetBase::setFilterQuads( const bool value )
{
    m_processor.setFilterQuads( value );
}

void CameraWidgetBase::setFastCheck( const bool value )
{
    m_processor.setFastCheck( value );
}

TemplateProcessor::Type CameraWidgetBase::templateType() const
{
    return m_processor.type();
}

const cv::Size &CameraWidgetBase::templateCount() const
{
    return m_processor.count();
}

double CameraWidgetBase::templateSize() const
{
    return m_processor.size();
}

bool CameraWidgetBase::resizeFlag() const
{
    return m_processor.resizeFlag();
}

unsigned int CameraWidgetBase::frameMaximumFlag() const
{
    return m_processor.frameMaximumFlag();
}

bool CameraWidgetBase::adaptiveThreshold() const
{
    return m_processor.adaptiveThreshold();
}

bool CameraWidgetBase::normalizeImage() const
{
    return m_processor.normalizeImage();
}

bool CameraWidgetBase::filterQuads() const
{
    return m_processor.filterQuads();
}

bool CameraWidgetBase::fastCheck() const
{
    return m_processor.fastCheck();
}

// MonocularCameraWidget
MonocularCameraWidget::MonocularCameraWidget( const std::string &cameraIp, QWidget* parent )
    : CameraWidgetBase( parent ), m_system(AVT::VmbAPI::VimbaSystem::GetInstance())
{
    initialize( cameraIp );
}

void MonocularCameraWidget::initialize(const std::string &cameraIp )
{
    checkVimbaStatus(m_system.Startup(), "Could not start Vimba system");

    checkVimbaStatus(m_system.OpenCameraByID( cameraIp.c_str(), VmbAccessModeFull, m_camera ),
        std::string( "Could not start open camera; ip = " ) + cameraIp );

    m_previewWidget = new PreviewWidget( this );
    addWidget( m_previewWidget );
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

void MonocularCameraWidget::setDecimation( const DecimationType type )
{
    int decimationFactor = static_cast<int>( type );

    setVimbaFeature( m_camera, "DecimationHorizontal",  decimationFactor );
    setVimbaFeature( m_camera, "DecimationVertical",  decimationFactor );
    setVimbaFeature( m_camera, "OffsetX",  0 );
    setVimbaFeature( m_camera, "OffsetY",  0 );
    setVimbaFeature( m_camera, "Width",  m_originalFrameSize / decimationFactor );
    setVimbaFeature( m_camera, "Height", m_originalFrameSize / decimationFactor );

}

void MonocularCameraWidget::updatePreview()
{
    auto frame = sourceImage();

    if (!frame.empty()) {
        CvImage procFrame;
        std::vector< cv::Point2f > previewPoints;

        m_processor.processPreview( frame, &procFrame, &previewPoints );

        setPreviewImage( procFrame );
        setPreviewPoints( previewPoints );

    }

}

void MonocularCameraWidget::timerEvent( QTimerEvent *event )
{
    CvImage frame;

    AVT::VmbAPI::FramePtr pFrame;
    VmbUchar_t *pImage;
    VmbUint32_t timeout = 500;
    VmbUint32_t nWidth = 0;
    VmbUint32_t nHeight = 0;

    VmbFrameStatusType status;

    checkVimbaStatus( m_camera->AcquireSingleImage( pFrame, timeout ), "FAILED to aquire frame!" );
    checkVimbaStatus( pFrame->GetReceiveStatus(status), "FAILED to aquire frame!" );
    checkVimbaStatus( pFrame->GetWidth( nWidth ), "FAILED to aquire width of frame!" );
    checkVimbaStatus( pFrame->GetHeight( nHeight ), "FAILED to aquire height of frame!" );
    checkVimbaStatus( pFrame->GetImage( pImage ), "FAILED to acquire image data of frame!" );

    auto mat = cv::Mat( nHeight, nWidth, CV_8UC3, pImage );
    mat.copyTo( frame );

    if ( !frame.empty() ) {
        setSourceImage( frame );

        updatePreview();

    }

}

// StereoCameraWidget
StereoCameraWidget::StereoCameraWidget(const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget* parent )
    : CameraWidgetBase( parent ), m_system(AVT::VmbAPI::VimbaSystem::GetInstance())
{
    initialize( leftCameraIp, rightCameraIp );
}

StereoCameraWidget::~StereoCameraWidget()
{
    m_system.Shutdown();
}

void StereoCameraWidget::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    checkVimbaStatus(m_system.Startup(), "Could not start Vimba system");

    checkVimbaStatus(m_system.OpenCameraByID( leftCameraIp.c_str(), VmbAccessModeFull, m_leftCamera ),
        "Could not start open camera; ip = " + leftCameraIp );

    checkVimbaStatus(m_system.OpenCameraByID( rightCameraIp.c_str(), VmbAccessModeFull, m_rightCamera ),
        "Could not start open camera; ip = " + rightCameraIp );

    setVimbaFeature( m_leftCamera, "PixelFormat", VmbPixelFormatBgr8 );
    setVimbaFeature( m_rightCamera, "PixelFormat", VmbPixelFormatBgr8 );

    m_leftCameraWidget = new PreviewWidget( this );
    m_rightCameraWidget = new PreviewWidget( this );

    addWidget( m_leftCameraWidget );
    addWidget( m_rightCameraWidget );

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

CvImage StereoCameraWidget::makeOverlappedPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage )
{
    return makePreview( leftPreviewImage, rightPreviewImage, 0.5 );

}

CvImage StereoCameraWidget::makeStraightPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage )
{
    return makePreview( leftPreviewImage, rightPreviewImage, 1 );
}

bool StereoCameraWidget::isTemplateExist() const
{
    return m_leftCameraWidget->isTemplateExist() || m_rightCameraWidget->isTemplateExist();
}

void StereoCameraWidget::setDecimation( const DecimationType type )
{
    int decimationFactor = static_cast<int>( type );

    setVimbaFeature( m_leftCamera, "DecimationHorizontal",  decimationFactor );
    setVimbaFeature( m_leftCamera, "DecimationVertical",  decimationFactor );
    setVimbaFeature( m_leftCamera, "OffsetX",  0 );
    setVimbaFeature( m_leftCamera, "OffsetY",  0 );
    setVimbaFeature( m_leftCamera, "Width",  m_originalFrameSize / decimationFactor );
    setVimbaFeature( m_leftCamera, "Height",  m_originalFrameSize / decimationFactor );

    setVimbaFeature( m_rightCamera, "DecimationHorizontal",  decimationFactor );
    setVimbaFeature( m_rightCamera, "DecimationVertical",  decimationFactor );
    setVimbaFeature( m_rightCamera, "OffsetX",  0 );
    setVimbaFeature( m_rightCamera, "OffsetY",  0 );
    setVimbaFeature( m_rightCamera, "Width",  m_originalFrameSize / decimationFactor );
    setVimbaFeature( m_rightCamera, "Height",  m_originalFrameSize / decimationFactor );

}

CvImage StereoCameraWidget::makePreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage, const double factor )
{
    CvImage result( std::max( leftPreviewImage.height(), rightPreviewImage.height() ),
                    leftPreviewImage.width() * factor + rightPreviewImage.width(),
                    leftPreviewImage.type(), cv::Scalar( 0, 0, 0, 0) );

    leftPreviewImage.copyTo( result( cv::Rect( 0, 0, leftPreviewImage.width(), leftPreviewImage.height() ) ) );
    rightPreviewImage.copyTo( result( cv::Rect( result.width() - rightPreviewImage.width(), 0, rightPreviewImage.width(), rightPreviewImage.height() ) ) );

    return result;

}

void StereoCameraWidget::updatePreview()
{
    updateLeftPreview();
    updateRightPreview();
}

void StereoCameraWidget::updateLeftPreview()
{
    CvImage frame = m_leftCameraWidget->sourceImage();

    if ( !frame.empty() ) {
        CvImage procFrame;
        std::vector< cv::Point2f > previewPoints;

        m_processor.processPreview( frame, &procFrame, &previewPoints );

        m_leftCameraWidget->setPreviewImage( procFrame );
        m_leftCameraWidget->setPreviewPoints( previewPoints );

    }

}

void StereoCameraWidget::updateRightPreview()
{
    CvImage frame = m_rightCameraWidget->sourceImage();

    if ( !frame.empty() ) {
        CvImage procFrame;
        std::vector< cv::Point2f > previewPoints;

        m_processor.processPreview( frame, &procFrame, &previewPoints );

        m_rightCameraWidget->setPreviewImage( procFrame );
        m_rightCameraWidget->setPreviewPoints( previewPoints );

    }

}

void StereoCameraWidget::timerEvent( QTimerEvent * )
{
    CvImage leftFrame;
    CvImage rightFrame;

    AVT::VmbAPI::FramePtr leftPFrame;
    AVT::VmbAPI::FramePtr rightPFrame;
    VmbUchar_t *leftPImage;
    VmbUchar_t *rightPImage;
    VmbUint32_t timeout = 500;
    VmbUint32_t nWidth = 0;
    VmbUint32_t nHeight = 0;

    VmbFrameStatusType status;

    checkVimbaStatus( m_leftCamera->AcquireSingleImage( leftPFrame, timeout ), "FAILED to aquire frame!" );
    checkVimbaStatus( m_rightCamera->AcquireSingleImage( rightPFrame, timeout ), "FAILED to aquire frame!" );

    checkVimbaStatus( leftPFrame->GetReceiveStatus( status ), "FAILED to aquire frame!" );
    checkVimbaStatus( rightPFrame->GetReceiveStatus( status ), "FAILED to aquire frame!" );

    checkVimbaStatus( leftPFrame->GetWidth( nWidth ), "FAILED to aquire width of frame!" );
    checkVimbaStatus( rightPFrame->GetWidth( nWidth ), "FAILED to aquire width of frame!" );

    checkVimbaStatus( leftPFrame->GetHeight( nHeight ), "FAILED to aquire height of frame!" );
    checkVimbaStatus( rightPFrame->GetHeight( nHeight ), "FAILED to aquire height of frame!" );

    checkVimbaStatus( leftPFrame->GetImage( leftPImage ), "FAILED to acquire image data of frame!" );
    checkVimbaStatus( rightPFrame->GetImage( rightPImage ), "FAILED to acquire image data of frame!" );

    auto leftMat = cv::Mat( nHeight, nWidth, CV_8UC3, leftPImage );
    leftMat.copyTo( leftFrame );

    auto rightMat = cv::Mat( nHeight, nWidth, CV_8UC3, rightPImage );
    rightMat.copyTo( rightFrame );

    if ( !leftFrame.empty() ) {
        m_leftCameraWidget->setSourceImage( leftFrame );
        updateLeftPreview();
    }

    if ( !rightFrame.empty() ) {
        m_rightCameraWidget->setSourceImage( rightFrame );
        updateRightPreview();

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

    m_cameraWidgets[0] = new PreviewWidget( this );
    m_cameraWidgets[1] = new PreviewWidget( this );
    m_cameraWidgets[2] = new PreviewWidget( this );

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

void TripleCameraWidget::timerEvent( QTimerEvent *event )
{

}

void TripleCameraWidget::updatePreview( const unsigned int cameraIndex )
{

}

