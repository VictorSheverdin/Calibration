#include "src/common/precompiled.h"

#include "reportwidget.h"

#include "calibrationiconswidget.h"
#include "camerawidget.h"
#include "src/common/functions.h"

#include "src/common/rectificationprocessor.h"

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

void ReportWidget::addIcon( const CalibrationIconBase& icon )
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

    auto previewImage = calibration.previewImage();

    if ( !previewImage.empty() ) {

        CvImage undistorted;

        cv::undistort( previewImage, undistorted, calibration.cameraMatrix(), calibration.distortionCoefficients() );

        CvImage resizedView = resizeTo( undistorted, m_reportFrameSize );

        addImage( resizedView );
        addSpace();

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

    if ( calibration.isOk() )
        addText( tr( "Calibration succesful!" ) + "\n" );
    else
        addText( tr( "Calibration error!" ) + "\n" );

}

const MonocularCalibrationData &MonocularReportWidget::calibrationResults() const
{
    return m_calibrationResults;
}

void MonocularReportWidget::saveYAMLDialog()
{
    if ( m_calibrationResults.isOk() ) {
        auto fileName = QFileDialog::getSaveFileName( nullptr, tr( "Save calibration file" ), QString(), tr( "OpenCV YAML files (*.yaml)" ), nullptr,
                                                      QFileDialog::DontUseNativeDialog );

        if ( !fileName.isEmpty() ) {
            m_calibrationResults.saveYaml( fileName.toStdString() );

        }

    }

}

void MonocularReportWidget::loadYAMLDialog()
{
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

    if ( calibration.isOk() ) {

        auto leftImage = calibration.leftCameraResults().previewImage();
        auto rightImage = calibration.rightCameraResults().previewImage();

        if ( !leftImage.empty() && !rightImage.empty() ) {

            StereoRectificationProcessor rectificationProcessor( calibration );

            CvImage leftRectifiedImage;
            CvImage rightRectifiedImage;
            CvImage leftCroppedImage;
            CvImage rightCroppedImage;

            if ( rectificationProcessor.rectify( leftImage, rightImage, &leftRectifiedImage, &rightRectifiedImage )
                        && rectificationProcessor.crop( leftRectifiedImage, rightRectifiedImage, &leftCroppedImage, &rightCroppedImage ) ) {

                auto leftResizedImage = resizeTo( leftRectifiedImage, m_reportFrameSize );
                auto rightResizedImage = resizeTo( rightRectifiedImage, m_reportFrameSize );

                auto stitchedImage = makeStraightPreview( leftResizedImage, rightResizedImage );

                if ( !stitchedImage.empty() ) {
                    drawTraceLines( stitchedImage, 15 );

                    addImage( stitchedImage );
                    addSpace();

                }

            }

        }

        addDoubleBreak();

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

        addText( tr( "Calibration succesful!" ) + "\n" );

    }
    else
        addText( tr( "Calibration error!" ) + "\n" );

}

const StereoCalibrationData &StereoReportWidget::calibrationResults() const
{
    return m_calibrationResults;
}

void StereoReportWidget::saveYAMLDialog()
{
    if ( m_calibrationResults.isOk() ) {
        auto fileName = QFileDialog::getSaveFileName( nullptr, tr( "Save calibration file" ), QString(), tr( "OpenCV YAML files (*.yaml)" ), nullptr,
                                                      QFileDialog::DontUseNativeDialog );

        if ( !fileName.isEmpty() ) {
            m_calibrationResults.saveYaml( fileName.toStdString() );

        }

    }

}

void StereoReportWidget::loadYAMLDialog()
{
}

