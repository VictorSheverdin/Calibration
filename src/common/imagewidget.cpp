#include "precompiled.h"

#include "imagewidget.h"

ImageWidget::ImageWidget( QWidget* parent )
    : QWidget( parent )
{
    initialize();
}


ImageWidget::ImageWidget(const CvImage frame, QWidget* parent )
    : QWidget( parent )
{
    initialize();

    setImage( frame );
}

void ImageWidget::initialize()
{
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
}

void ImageWidget::setImage( const CvImage &image )
{
    m_image = image;

    repaint();
}

const CvImage ImageWidget::image() const
{
    return m_image;
}

int ImageWidget::imageWidth() const
{
    return m_image.width();
}

int ImageWidget::imageHeight() const
{
    return m_image.height();
}

double ImageWidget::imageAspect() const
{
    auto height = imageHeight();

    if ( height > 0 )
        return static_cast<double>( imageWidth() ) / height;
    else
        return 0.0;

}

void ImageWidget::paintEvent(QPaintEvent *event )
{
    QWidget::paintEvent( event );

    QPainter painter( this );

    painter.setRenderHint( QPainter::HighQualityAntialiasing );

    if ( !m_image.empty() ) {
        auto image = static_cast< QtImage >( m_image );

        auto scaledImage = image.scaled( size(), Qt::KeepAspectRatio, Qt::SmoothTransformation );

        painter.drawImage( rect().center() - scaledImage.rect().center(), scaledImage );

    }

}

void ImageWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent( event );

    emit resizeSignal();

}

// ImageViewer
ImageViewer::ImageViewer( QWidget *parent )
    : QScrollArea( parent )
{
    initialize();
}

void ImageViewer::initialize()
{
    m_scaleFactor = 1.;

    m_image = new ImageWidget( this );
    setBackgroundRole( QPalette::Dark );

    setWidget( m_image );
}

void ImageViewer::setImage( const CvImage &image )
{
    m_image->setImage( image );

    updateScale();
}

void ImageViewer::zoomIn()
{
    scaleImage( 1.25 );
}

void ImageViewer::zoomOut()
{
     scaleImage( 0.75 );
}

void ImageViewer::normalSize()
{
    setScale( 1. );
}

void ImageViewer::scaleImage( double factor )
{
    setScale( std::max( 0.1, std::min( 5., m_scaleFactor * factor ) ) );
}

void ImageViewer::setScale( const double value )
{
    if ( value > 0 ) {
        m_scaleFactor = value;
        updateScale();

    }
}

void ImageViewer::updateScale()
{
    m_image->resize( m_image->imageWidth() * m_scaleFactor, m_image->imageHeight() * m_scaleFactor );
}

void ImageViewer::wheelEvent( QWheelEvent *event )
{
    double dScale = 1. + event->angleDelta().ry() / 480.;

    scaleImage( dScale );

    event->accept();
}

// ImageDialog
ImageDialog::ImageDialog( QWidget *parent )
    : DialogBase( parent )
{
    initialize();
}

void ImageDialog::initialize()
{
    setWindowTitle( tr( "Image Viewer" ) );

    setWidget( new ImageWidget( this ) );

}

ImageWidget *ImageDialog::widget() const
{
    return dynamic_cast< ImageWidget * >( m_widget.data() );
}

void ImageDialog::setImage(const CvImage image)
{
    widget()->setImage( image );
}
