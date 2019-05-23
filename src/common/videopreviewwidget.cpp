#include "precompiled.h"

#include "videopreviewwidget.h"

#include "defs.h"
#include "functions.h"

#include "image.h"

// VideoIconWidget
VideoIconWidget::VideoIconWidget( QWidget* parent )
    : ImageWidget( parent )
{
    initialize();
}

VideoIconWidget::VideoIconWidget( const CvImage &icon, QWidget* parent )
    : ImageWidget( parent )
{
    initialize();

    setImage( icon );
}

void VideoIconWidget::initialize()
{
    setMouseTracking( true );
    setAttribute( Qt::WA_Hover );

}

void VideoIconWidget::paintEvent( QPaintEvent *event )
{
    ImageWidget::paintEvent( event );

    if ( underMouse() ) {

        QStylePainter painter(this);

        painter.setRenderHint( QPainter::HighQualityAntialiasing );

        QStyleOptionFocusRect option;
        option.initFrom(this);
        option.backgroundColor = palette().color( QPalette::Highlight );

        painter.drawPrimitive( QStyle::PE_Frame, option );

    }

}

void VideoIconWidget::mouseDoubleClickEvent( QMouseEvent * )
{
    emit iconActivated( this );
}

// VideoIconsWidget
VideoIconsLayout::VideoIconsLayout( QWidget *parent )
    : QWidget( parent )
{
    initialize();
}

void VideoIconsLayout::initialize()
{
    m_layout = new QBoxLayout( QBoxLayout::LeftToRight, this );

    setLayout( m_layout );
}

void VideoIconsLayout::setOrientation( const Qt::Orientation value )
{
    if ( value == Qt::Horizontal )
        m_layout->setDirection( QBoxLayout::LeftToRight );
    else
        m_layout->setDirection( QBoxLayout::TopToBottom );

}

Qt::Orientation VideoIconsLayout::orientation() const
{
    if ( m_layout->direction() == QBoxLayout::LeftToRight || m_layout->direction() == QBoxLayout::RightToLeft )
        return Qt::Horizontal;
    else
        return Qt::Vertical;
}

void VideoIconsLayout::addIcon( VideoIconWidget *icon )
{
    connect( icon, &VideoIconWidget::iconActivated, this, &VideoIconsLayout::iconActivated );

    m_layout->addWidget( icon );


}

void VideoIconsLayout::addIcons( const std::vector< VideoIconWidget * > &icons )
{
    for ( auto i = icons.begin(); i != icons.end(); ++i )
        addIcon( *i );
}

unsigned int VideoIconsLayout::framesCount() const
{
    return m_layout->count();
}

double VideoIconsLayout::maximumAspectRatio() const
{
    double ret = 0.0;

    for ( auto i = 0; i < m_layout->count(); ++i ) {
        auto frame = frameAt( i );

        if (frame) {
            auto aspect = frame->imageAspect();

            ret = std::max( 0.0, aspect );
        }
    }

    return ret;

}

VideoIconWidget *VideoIconsLayout::frameAt( const size_t i ) const
{
    return dynamic_cast< VideoIconWidget *> ( m_layout->itemAt( i )->widget() );
}

void VideoIconsLayout::clear()
{
    QLayoutItem *child;

    while ( ( child = m_layout->takeAt( 0 ) ) != nullptr ) {
        delete child->widget();
        delete child;
    }

}

// VideoIconsWidget
VideoIconsWidget::VideoIconsWidget( QWidget *parent )
    : QScrollArea( parent )
{
    initialize();

}

void VideoIconsWidget::initialize()
{
    auto palette = this->palette();
    palette.setColor( QPalette::Background, Qt::white );
    setPalette( palette );

    setAutoFillBackground( true );

    auto layout = new VideoIconsLayout( this );
    setWidget( layout );

    connect( layout, &VideoIconsLayout::iconActivated, this, &VideoIconsWidget::iconActivated );

    setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );

}

VideoIconsLayout *VideoIconsWidget::layoutWidget() const
{
    return dynamic_cast<VideoIconsLayout *>( widget() );
}

void VideoIconsWidget::setIcons( const std::vector<VideoIconWidget *> &icons )
{
    auto layout = layoutWidget();

    layout->clear();

    layout->addIcons( icons );

    updateLayout();

}

void VideoIconsWidget::addIcon( VideoIconWidget *icon )
{
    auto layout = layoutWidget();

    layout->addIcon( icon );

    updateLayout();

}

void VideoIconsWidget::addIcons( const std::vector< VideoIconWidget* > &icons )
{
    auto layout = layoutWidget();

    layout->addIcons( icons );

    updateLayout();

}

void VideoIconsWidget::setOrientation( const Qt::Orientation value )
{
    layoutWidget()->setOrientation( value );

    updateLayout();

}

Qt::Orientation VideoIconsWidget::orientation() const
{
    return layoutWidget()->orientation();
}

void VideoIconsWidget::updateLayout()
{
    auto layout = layoutWidget();

    int width;
    int height;

    if ( orientation() == Qt::Horizontal ) {
        height = viewport()->height();
        width = layout->framesCount() * layout->maximumAspectRatio() * height;

    }
    else {

        width = viewport()->width();

        if ( layout->maximumAspectRatio() > DOUBLE_EPS )
            height = layout->framesCount() * width / layout->maximumAspectRatio();
        else
            height = 0;

    }

    layout->setMinimumSize( width, height );
    layout->setMaximumSize( width, height );

}

void VideoIconsWidget::resizeEvent(QResizeEvent */*event*/)
{
    updateLayout();
}

