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

void ImageWidget::setImage(const CvImage image)
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

// ImageDialog
ImageDialog::ImageDialog( QWidget *parent )
    : QDialog( parent )
{
    initialize();
}

void ImageDialog::initialize()
{
    setWindowTitle( tr( "Image Viewer" ) );

    m_imageWidget = new ImageWidget( this );

    QVBoxLayout *layout = new QVBoxLayout( this );

    layout->addWidget( m_imageWidget );

}

void ImageDialog::setImage(const CvImage image)
{
    m_imageWidget->setImage( image );
}
