#include "precompiled.h"

#include "reportwidget.h"

#include "iconswidget.h"

// ReportWidget
ReportWidget::ReportWidget( QWidget* parent )
    : QTextEdit( parent )
{
    initialize();
}

void ReportWidget::initialize()
{
    setReadOnly( true );
}

void ReportWidget::addText( const QString &text )
{
    textCursor().insertText( text );
}

void ReportWidget::addBreak()
{
    addText( "\n" );
}

void ReportWidget::addDoubleBreak()
{
    addBreak();
    addBreak();
}

void ReportWidget::addNumber( const int num )
{
    addText( QString::number( num ) );
}

void ReportWidget::addNumber( const double num )
{
    addText( QString::number( num ) );
}

void ReportWidget::addIcon( const IconBase& icon )
{
    addImage( icon.previewImage() );
}

void ReportWidget::addImage( const CvImage& image )
{
    textCursor().insertImage( QtImage( image ) );
}

void ReportWidget::addMatrix( const cv::Mat& mat )
{
    QString html;

    html += "<table>";

    for ( auto i = 0; i < mat.rows; ++i ) {
        html += "<tr>";
        for ( auto j = 0; j <mat.cols; ++j ) {
            html += "<td>";
            html += QString::number( mat.at<double>( i, j ) );
            html += "</td>";
        }
        html += "</tr>";
    }

    html +="</table>";

    insertHtml( html );

}

void ReportWidget::addRect( const cv::Rect& rect )
{
    addText( "x:" );
    addNumber( rect.x );
    addText( " " );

    addText( "y:" );
    addNumber( rect.y );
    addText( " " );

    addText( "width:" );
    addNumber( rect.width );
    addText( " " );

    addText( "height:" );
    addNumber( rect.height );
    addText( " " );
}

// ReportDialog
ReportDialog::ReportDialog( QWidget *parent )
    : QDialog( parent )
{
    initialize();
}

void ReportDialog::initialize()
{
    setWindowTitle( tr( "Report" ) );

    QVBoxLayout *layout = new QVBoxLayout( this );

    m_reportWidget = new ReportWidget( this );

    layout->addWidget( m_reportWidget );
}

void ReportDialog::clear()
{
    m_reportWidget->clear();
}

void ReportDialog::addText( const QString &text )
{
    m_reportWidget->addText( text );
}

void ReportDialog::addBreak()
{
    m_reportWidget->addBreak();
}

void ReportDialog::addDoubleBreak()
{
    m_reportWidget->addDoubleBreak();
}

void ReportDialog::addNumber( const int num )
{
    m_reportWidget->addNumber( num );
}

void ReportDialog::addNumber( const double num )
{
    m_reportWidget->addNumber( num );
}

void ReportDialog::addIcon( const IconBase& icon )
{
    m_reportWidget->addIcon( icon );
}

void ReportDialog::addImage( const CvImage& image )
{
    m_reportWidget->addImage( image );
}

void ReportDialog::addMatrix( const cv::Mat& mat )
{
    m_reportWidget->addMatrix( mat );
}

void ReportDialog::addRect( const cv::Rect& rect )
{
    m_reportWidget->addRect( rect );
}


