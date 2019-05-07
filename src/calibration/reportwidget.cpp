#include "src/common/precompiled.h"

#include "reportwidget.h"

#include "iconswidget.h"
#include "camerawidget.h"
#include "src/common/functions.h"

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

void ReportWidget::addSpace( const int num )
{
    for ( auto i = 0; i < num; ++i )
        addText( " " );
}

void ReportWidget::addIcon( const IconBase& icon )
{
    addImage( icon.previewImage() );
}

void ReportWidget::addImage( const CvImage& image )
{
    textCursor().insertImage( QtImage( image ) );
}

void ReportWidget::addSize( const cv::Size& size )
{
    addText( "width:" );
    addNumber( size.width );
    addText( " " );

    addText( "height:" );
    addNumber( size.height );
    addText( " " );

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

// MonocularReportWidge
MonocularReportWidget::MonocularReportWidget( QWidget *parent )
    : ReportWidget( parent )
{
}

void MonocularReportWidget::report( const MonocularCalibrationData &calibration )
{
    m_calibrationResults = calibration;

    clear();

    addText( tr( "Camera calibration." ) + "\n" );
    addBreak();


    for (  auto &i : calibration.results() ) {

        if ( !i.processedView().empty() ) {

            CvImage resizedView = resizeTo( i.processedView(), m_reportFrameSize );

            addImage( resizedView );
            addSpace();


        }

    }

    addDoubleBreak();

    addText( tr( "Results:" ) + "\n" );
    addBreak();

    addText( tr( "Processed images count: " ) );
    addNumber( calibration.resultsSize() );
    addDoubleBreak();

    addText( tr( "Image size " ) );
    addSize( calibration.frameSize() );
    addDoubleBreak();

    addText( tr( "Camera matrix:" ) + "\n" );
    addMatrix( calibration.cameraMatrix() );
    addBreak();

    addBreak();
    addText( tr( "Distorsion coefficients:" ) + " " );
    addMatrix( calibration.distortionCoefficients().t() );
    addBreak();

    addBreak();
    addText( tr( "Reprojection error:" ) +  " " + QString::number( calibration.error() ) +"\n" );
    addBreak();

    for ( auto &i : calibration.results() ) {

        if ( !i.sourceView().empty() ) {

            CvImage undistorted;

            cv::undistort( i.sourceView(), undistorted, calibration.cameraMatrix(), calibration.distortionCoefficients() );

            CvImage resizedView = resizeTo( undistorted, m_reportFrameSize );

            addImage( resizedView );
            addSpace();

        }

    }

    addDoubleBreak();

    if ( calibration.isOk() )
        addText( tr( "Calibration succesful!" ) + "\n" );
    else
        addText( tr( "Calibration error!" ) + "\n" );

}

const MonocularCalibrationData &MonocularReportWidget::calibrationResults() const
{
    return m_calibrationResults;
}

// StereoReportWidget
StereoReportWidget::StereoReportWidget( QWidget *parent )
    : ReportWidget( parent )
{
}

void StereoReportWidget::report(const StereoCalibrationData &calibration )
{
    m_calibrationResults = calibration;

    clear();

    addText( tr( "Stereo camera calibration." ) + "\n" );
    addBreak();

    if ( calibration.leftResultsSize() != calibration.rightResultsSize() )
        throw std::exception();

    for (  int i = 0; i < calibration.leftCameraResults().resultsSize(); ++i ) {
        if ( calibration.leftCameraResults().result(i).isOk() &&  calibration.rightCameraResults().result(i).isOk() ) {

            auto image1 = resizeTo( calibration.leftCameraResults().result( i ).processedView(), m_reportFrameSize );
            auto image2 = resizeTo( calibration.rightCameraResults().result( i ).processedView(), m_reportFrameSize );

            auto stitchedImage = StereoCameraWidget::makeStraightPreview( image1, image2 );

            if ( !stitchedImage.empty() ) {
                addImage( stitchedImage );
                addSpace();
            }

        }

    }

    addDoubleBreak();

    if ( calibration.isOk() ) {

        addText( tr( "Left camera matrix:" ) + "\n" );
        addMatrix( calibration.leftCameraResults().cameraMatrix() );
        addDoubleBreak();

        addText( tr( "Right camera matrix:" ) + "\n" );
        addMatrix( calibration.rightCameraResults().cameraMatrix() );
        addDoubleBreak();

        addText( tr( "Left distorsion coefficients:" ) + " " );
        addMatrix( calibration.leftCameraResults().distortionCoefficients().t() );
        addDoubleBreak();

        addText( tr( "Right distorsion coefficients:" ) + " " );
        addMatrix( calibration.rightCameraResults().distortionCoefficients().t() );
        addDoubleBreak();

        addText( tr( "Rotation matrix:" ) + " " );
        addMatrix( calibration.rotationMatrix() );
        addDoubleBreak();

        addText( tr( "Translation vector:" ) + " " );
        addMatrix( calibration.translationVector().t() );
        addDoubleBreak();

        addText( tr( "Fundamental matrix:" ) + " " );
        addMatrix( calibration.fundamentalMatrix() );
        addDoubleBreak();

        addText( tr( "Essential matrix:" ) + " " );
        addMatrix( calibration.essentialMatrix() );
        addDoubleBreak();

        addText( tr( "Left camera rectification matrix:" ) + " " );
        addMatrix( calibration.leftRectifyMatrix() );
        addDoubleBreak();

        addText( tr( "Left camera projection matrix:" ) + " " );
        addMatrix( calibration.leftProjectionMatrix() );
        addDoubleBreak();

        addText( tr( "Right camera rectification matrix:" ) + " " );
        addMatrix( calibration.rightRectifyMatrix() );
        addDoubleBreak();

        addText( tr( "Right camera projection matrix:" ) + " " );
        addMatrix( calibration.rightProjectionMatrix() );
        addDoubleBreak();

        addText( tr( "Left valid ROI:" ) + " " );
        addRect( calibration.leftROI() );
        addDoubleBreak();

        addText( tr( "Right valid ROI:" ) + " " );
        addRect( calibration.rightROI() );
        addDoubleBreak();

        addText( tr( "Reprojection error:" ) +  " " + QString::number( calibration.error() ) +"\n" );
        addBreak();

        for ( auto i = 0; i < calibration.leftCameraResults().resultsSize(); ++i ) {
            auto leftImage = calibration.leftCameraResults().result(i).sourceView();
            auto rightImage = calibration.rightCameraResults().result(i).sourceView();

            if ( !leftImage.empty() && !rightImage.empty() ) {

                cv::Mat leftRectifiedImage;
                cv::Mat rightRectifiedImage;

                cv::remap( leftImage, leftRectifiedImage, calibration.leftRMap(), calibration.leftDMap(), cv::INTER_LANCZOS4 );
                cv::remap( rightImage, rightRectifiedImage, calibration.rightRMap(), calibration.rightDMap(), cv::INTER_LANCZOS4 );

                auto image1 = resizeTo( leftRectifiedImage, m_reportFrameSize );
                auto image2 = resizeTo( rightRectifiedImage, m_reportFrameSize );

                auto stitchedImage = StereoCameraWidget::makeStraightPreview( image1, image2 );

                if ( !stitchedImage.empty() ) {
                    drawTraceLines( stitchedImage, 15 );

                    addImage( stitchedImage );
                    addSpace();

                }


            }

        }

        addDoubleBreak();

        addText( tr( "Calibration succesful!" ) + "\n" );

    }
    else
        addText( tr( "Calibration error!" ) + "\n" );

}

const StereoCalibrationData &StereoReportWidget::calibrationResults() const
{
    return m_calibrationResults;
}

// ReportDialogBase
ReportDialogBase::ReportDialogBase( QWidget *parent )
    : QDialog( parent )
{
    initialize();
}

void ReportDialogBase::initialize()
{
    setWindowTitle( tr( "Report" ) );

    m_layout = new QVBoxLayout( this );

    m_toolBar = new QToolBar( tr("Report"), this );

    m_exportYamlAction = new QAction( QIcon( ":/resources/images/xml.ico" ), tr( "Export YAML" ), this );
    m_exportOpenCVAction = new QAction( QIcon( ":/resources/images/opencv.ico" ), tr( "Export OpenCV" ) );

    m_toolBar->addAction( m_exportYamlAction );
    m_toolBar->addAction( m_exportOpenCVAction );

    m_layout->addWidget( m_toolBar );

}

// MonocularReportDialog
MonocularReportDialog::MonocularReportDialog( QWidget *parent )
    : ReportDialogBase( parent )
{
    initialize();
}

void MonocularReportDialog::initialize()
{
    m_reportWidget = new MonocularReportWidget( this );

    m_layout->addWidget( m_reportWidget );

    connect( m_exportOpenCVAction, &QAction::triggered, this, &MonocularReportDialog::saveYAMLDialog );

}

void MonocularReportDialog::report( const MonocularCalibrationData &calibration )
{
    m_reportWidget->report( calibration );
}

const MonocularCalibrationData &MonocularReportDialog::calibrationResults() const
{
    return m_reportWidget->calibrationResults();
}

void MonocularReportDialog::saveYAMLDialog()
{
    if ( m_reportWidget->calibrationResults().isOk() ) {
        auto fileName = QFileDialog::getSaveFileName( nullptr, tr( "Save calibration file" ), QString(), tr( "OpenCV YAML files (*.yaml)" ), nullptr,
                                                      QFileDialog::DontUseNativeDialog );

        if ( !fileName.isEmpty() ) {
            m_reportWidget->calibrationResults().saveYaml( fileName.toStdString() );

        }

    }

}

void MonocularReportDialog::loadYAMLDialog()
{
}

// StereoReportDialog
StereoReportDialog::StereoReportDialog( QWidget *parent )
    : ReportDialogBase( parent )
{
    initialize();
}

void StereoReportDialog::initialize()
{
    m_reportWidget = new StereoReportWidget( this );

    m_layout->addWidget( m_reportWidget );

    connect( m_exportOpenCVAction, &QAction::triggered, this, &StereoReportDialog::saveYAMLDialog );

}

void StereoReportDialog::report( const StereoCalibrationData &calibration )
{
    m_reportWidget->report( calibration );
}

const StereoCalibrationData &StereoReportDialog::calibrationResults() const
{
    return m_reportWidget->calibrationResults();
}

void StereoReportDialog::saveYAMLDialog()
{
    if ( m_reportWidget->calibrationResults().isOk() ) {
        auto fileName = QFileDialog::getSaveFileName( nullptr, tr( "Save calibration file" ), QString(), tr( "OpenCV YAML files (*.yaml)" ), nullptr,
                                                      QFileDialog::DontUseNativeDialog );

        if ( !fileName.isEmpty() ) {
            m_reportWidget->calibrationResults().saveYaml( fileName.toStdString() );

        }

    }

}

void StereoReportDialog::loadYAMLDialog()
{
}
