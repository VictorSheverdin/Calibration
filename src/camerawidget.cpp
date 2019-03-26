#include "precompiled.h"

#include "camerawidget.h"

#include "previewwidget.h"

// CameraWidgetBase
CameraWidgetBase::CameraWidgetBase( QWidget* parent )
    : QSplitter( Qt::Horizontal, parent )
{
    initialize();
}

void CameraWidgetBase::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );

    startTimer( 1 );
}

void CameraWidgetBase::setType( const TemplateProcessor::Type type )
{
    m_processor.setType( type );
}

void CameraWidgetBase::setCount( const cv::Size &count )
{
    m_processor.setCount( count );
}

void CameraWidgetBase::setSize( const double value )
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

TemplateProcessor::Type CameraWidgetBase::type() const
{
    return m_processor.type();
}

const cv::Size &CameraWidgetBase::count() const
{
    return m_processor.count();
}

double CameraWidgetBase::size() const
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

// MonocularCameraWidget
MonocularCameraWidget::MonocularCameraWidget(const int cameraIndex, QWidget* parent )
    : CameraWidgetBase( parent )
{
    initialize( cameraIndex );
}

void MonocularCameraWidget::initialize( const int cameraIndex )
{
    m_capture.open( cameraIndex );

    m_previewWidget = new PreviewWidget( this );
    addWidget( m_previewWidget );
}

void MonocularCameraWidget::setSourceImage(const CvImage image)
{
    m_previewWidget->setSourceImage( image );
}

void MonocularCameraWidget::setDisplayedImage(const CvImage image)
{
    m_previewWidget->setDisplayedImage( image );
}

const CvImage MonocularCameraWidget::sourceImage() const
{
    return m_previewWidget->sourceImage();
}

const CvImage MonocularCameraWidget::previewImage() const
{
    return m_previewWidget->previewImage();
}

void MonocularCameraWidget::updatePreview()
{
    auto frame = sourceImage();

    if (!frame.empty()) {
        CvImage procFrame;
        std::vector<cv::Point2f> points;

        m_processor.process( frame, &procFrame, &points );

        setDisplayedImage( procFrame );
    }

}

void MonocularCameraWidget::timerEvent( QTimerEvent *event )
{
    if ( m_capture.isOpened() ) {

        CvImage frame;

        m_capture >> frame;

        if ( !frame.empty() ) {
            setSourceImage( frame );

            updatePreview();

        }

    }

}

// StereoCameraWidget
StereoCameraWidget::StereoCameraWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget* parent )
    : CameraWidgetBase( parent )
{
    initialize( leftCameraIndex, rightCameraIndex );
}

void StereoCameraWidget::initialize( const int leftCameraIndex, const int rightCameraIndex )
{
    m_leftCapture.open( leftCameraIndex );
    m_rightCapture.open( rightCameraIndex );

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
    return m_leftCameraWidget->setDisplayedImage( image );
}

void StereoCameraWidget::setRightSourceImage( const CvImage image )
{
    return m_rightCameraWidget->setSourceImage( image );
}

void StereoCameraWidget::setRightDisplayedImage( const CvImage image )
{
    return m_rightCameraWidget->setDisplayedImage( image );
}

CvImage StereoCameraWidget::createPreview( const CvImage &leftPreviewImage, const CvImage &rightPreviewImage )
{
    CvImage result( std::max( leftPreviewImage.height(), rightPreviewImage.height() ),
                    leftPreviewImage.width() / 2 + rightPreviewImage.width(),
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
        std::vector<cv::Point2f> points;

        m_processor.process( frame, &procFrame, &points );

        m_leftCameraWidget->setDisplayedImage( procFrame );

    }

}

void StereoCameraWidget::updateRightPreview()
{
    CvImage frame = m_rightCameraWidget->sourceImage();

    if ( !frame.empty() ) {
        CvImage procFrame;
        std::vector<cv::Point2f> points;

        m_processor.process( frame, &procFrame, &points );

        m_rightCameraWidget->setDisplayedImage( procFrame );

    }

}

void StereoCameraWidget::timerEvent(QTimerEvent *event)
{
    if ( m_leftCapture.isOpened() && m_rightCapture.isOpened() ) {

        CvImage leftFrame;
        CvImage rightFrame;

        m_leftCapture >> leftFrame;
        m_rightCapture >> rightFrame;

        if ( !leftFrame.empty() ) {
            m_leftCameraWidget->setSourceImage( leftFrame );
            updateLeftPreview();

        }

        if ( !rightFrame.empty() ) {
            m_rightCameraWidget->setSourceImage( rightFrame );
            updateRightPreview();

        }

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

void TripleCameraWidget::setDisplayedImage( const unsigned int cameraIndex, const CvImage image )
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

