#include "precompiled.h"

#include "reportwidget.h"

#include "iconswidget.h"

#include "camerawidget.h"
#include "functions.h"

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

// MonocularCalibrationResults
MonocularCalibrationResult::MonocularCalibrationResult()
{
    initialize();
}

void MonocularCalibrationResult::initialize()
{
    m_ok = false;
}

void MonocularCalibrationResult::setSourceView( const CvImage &value )
{
    m_sourceView = value;
}

const CvImage &MonocularCalibrationResult::sourceView() const
{
    return m_sourceView;
}

void MonocularCalibrationResult::setProcessedView( const CvImage &value )
{
    m_processedView = value;
}

const CvImage &MonocularCalibrationResult::processedView() const
{
    return m_processedView;
}

void MonocularCalibrationResult::setRVec( const cv::Mat &value )
{
    m_rVec = value;
}

const cv::Mat &MonocularCalibrationResult::rVec() const
{
    return m_rVec;
}

void MonocularCalibrationResult::setTVec( const cv::Mat &value )
{
    m_tVec = value;
}

const cv::Mat &MonocularCalibrationResult::tVec() const
{
    return m_tVec;
}

void MonocularCalibrationResult::setPoints2d( const std::vector< cv::Point2f > &value )
{
    m_points2d = value;
}

const std::vector< cv::Point2f > &MonocularCalibrationResult::points2d() const
{
    return m_points2d;
}

void MonocularCalibrationResult::setPoints3d( const std::vector< cv::Point3f > &value )
{
    m_points3d = value;
}

const std::vector< cv::Point3f > &MonocularCalibrationResult::points3d() const
{
    return m_points3d;
}

void MonocularCalibrationResult::setOk( const bool value )
{
    m_ok = value;
}

bool MonocularCalibrationResult::isOk() const
{
    return m_ok;
}

// MonocularCalibrationResults
MonocularCalibrationResults::MonocularCalibrationResults()
{
    initialize();
}

void MonocularCalibrationResults::initialize()
{
    m_ok = false;
}

void MonocularCalibrationResults::setFrameSize( const cv::Size &value )
{
    m_frameSize = value;
}

const cv::Size &MonocularCalibrationResults::frameSize() const
{
    return m_frameSize;
}

void MonocularCalibrationResults::setCameraMatrix( const cv::Mat &value )
{
    m_cameraMatrix = value;
}

const cv::Mat &MonocularCalibrationResults::cameraMatrix() const
{
    return m_cameraMatrix;
}

void MonocularCalibrationResults::setDistorsionCoefficients( const cv::Mat &value )
{
    m_distCoefficients = value;
}

const cv::Mat &MonocularCalibrationResults::distorsionCoefficients() const
{
    return m_distCoefficients;
}

void MonocularCalibrationResults::setResults( std::vector< MonocularCalibrationResult > &value )
{
    m_results = value;
}

const std::vector< MonocularCalibrationResult > &MonocularCalibrationResults::results() const
{
    return m_results;
}

MonocularCalibrationResult &MonocularCalibrationResults::result( const unsigned int i )
{
    return m_results[i];
}

const MonocularCalibrationResult &MonocularCalibrationResults::result( const unsigned int i ) const
{
    return m_results[i];
}

void MonocularCalibrationResults::setOk( const bool value )
{
    m_ok = value;
}

bool MonocularCalibrationResults::isOk() const
{
    return m_ok;
}

void MonocularCalibrationResults::setError( const double value )
{
    m_error = value;
}

double MonocularCalibrationResults::error() const
{
    return m_error;
}

const unsigned int MonocularCalibrationResults::resultsSize() const
{
    return m_results.size();
}


bool MonocularCalibrationResults::saveYaml( const std::string &fileName ) const
{
    if ( isOk() ) {
        cv::FileStorage fs( fileName, cv::FileStorage::WRITE );

        fs << "frameCount" << static_cast<int>( resultsSize() );

        time_t rawtime; time( &rawtime );
        fs << "calibrationDate" << asctime( localtime( &rawtime ) );

        fs << "cameraMatrix" << cameraMatrix();
        fs << "distortionCoefficients" << distorsionCoefficients();

        fs.release();
    }
}

bool MonocularCalibrationResults::loadYaml( const std::string &fileName )
{

}

// StereoCalibrationResults
StereoCalibrationResults::StereoCalibrationResults()
{
}

void StereoCalibrationResults::setLeftCameraResults( const MonocularCalibrationResults &value )
{
    m_leftCameraResults = value;
}

const MonocularCalibrationResults &StereoCalibrationResults::leftCameraResults() const
{
    return m_leftCameraResults;
}

void StereoCalibrationResults::setRightCameraResults( const MonocularCalibrationResults &value )
{
    m_rightCameraResults = value;
}

const MonocularCalibrationResults &StereoCalibrationResults::rightCameraResults() const
{
    return m_rightCameraResults;
}

unsigned int StereoCalibrationResults::leftResultsSize() const
{
    return m_leftCameraResults.resultsSize();
}

unsigned int StereoCalibrationResults::rightResultsSize() const
{
    return m_rightCameraResults.resultsSize();
}

void StereoCalibrationResults::setRotationMatrix( const cv::Mat &value )
{
    m_rotationMatrix = value;
}

const cv::Mat &StereoCalibrationResults::rotationMatrix() const
{
    return m_rotationMatrix;
}

void StereoCalibrationResults::setTranslationVector( const cv::Mat &value )
{
    m_translationVector = value;
}

const cv::Mat &StereoCalibrationResults::translationVector() const
{
    return m_translationVector;
}

void StereoCalibrationResults::setFundamentalMatrix( const cv::Mat &value )
{
    m_fundamentalMatrix = value;
}

const cv::Mat &StereoCalibrationResults::fundamentalMatrix() const
{
    return m_fundamentalMatrix;
}

void StereoCalibrationResults::setEssentialMatrix( const cv::Mat &value )
{
    m_essentialMatrix = value;
}

const cv::Mat &StereoCalibrationResults::essentialMatrix() const
{
    return m_essentialMatrix;
}

void StereoCalibrationResults::setLeftRectifyMatrix( const cv::Mat &value )
{
    m_leftRectifyMatrix = value;
}

const cv::Mat &StereoCalibrationResults::leftRectifyMatrix() const
{
    return m_leftRectifyMatrix;
}

void StereoCalibrationResults::setRightRectifyMatrix( const cv::Mat &value )
{
    m_rightRectifyMatrix = value;
}

const cv::Mat &StereoCalibrationResults::rightRectifyMatrix() const
{
    return m_rightRectifyMatrix;
}

void StereoCalibrationResults::setLeftProjectionMatrix( const cv::Mat &value )
{
    m_leftProjectionMatrix = value;
}

const cv::Mat &StereoCalibrationResults::leftProjectionMatrix() const
{
    return m_leftProjectionMatrix;
}

void StereoCalibrationResults::setRightProjectionMatrix( const cv::Mat &value )
{
    m_rightProjectionMatrix = value;
}

const cv::Mat &StereoCalibrationResults::rightProjectionMatrix() const
{
    return m_rightProjectionMatrix;
}

void StereoCalibrationResults::setDisparityToDepthMatrix( const cv::Mat &value )
{
    m_disparityToDepthMatrix = value;
}

const cv::Mat &StereoCalibrationResults::disparityToDepthMatrix() const
{
    return m_disparityToDepthMatrix;
}

void StereoCalibrationResults::setLeftROI( const cv::Rect &value )
{
    m_leftROI = value;
}

const cv::Rect &StereoCalibrationResults::leftROI() const
{
    return m_leftROI;
}

void StereoCalibrationResults::setRightROI( const cv::Rect &value )
{
    m_rightROI = value;
}

const cv::Rect &StereoCalibrationResults::rightROI() const
{
    return m_rightROI;
}

void StereoCalibrationResults::setLeftRMap( const cv::Mat &value )
{
    m_leftRMap = value;
}

const cv::Mat &StereoCalibrationResults::leftRMap() const
{
    return m_leftRMap;
}

void StereoCalibrationResults::setLeftDMap( const cv::Mat &value )
{
    m_leftDMap = value;
}

const cv::Mat &StereoCalibrationResults::leftDMap() const
{
    return m_leftDMap;
}

void StereoCalibrationResults::setRightRMap( const cv::Mat &value )
{
    m_rightRMap = value;
}

const cv::Mat &StereoCalibrationResults::rightRMap() const
{
    return m_rightRMap;
}

void StereoCalibrationResults::setRightDMap( const cv::Mat &value )
{
    m_rightDMap = value;
}

const cv::Mat &StereoCalibrationResults::rightDMap() const
{
    return m_rightDMap;
}

void StereoCalibrationResults::setError( const double value )
{
    m_error = value;
}

double StereoCalibrationResults::error() const
{
    return m_error;
}

bool StereoCalibrationResults::isOk() const
{
    return m_leftCameraResults.isOk() && m_rightCameraResults.isOk();
}

bool StereoCalibrationResults::saveYaml( std::string &fileName ) const
{

}

bool StereoCalibrationResults::loadYaml(std::string &fileName )
{

}

// MonocularReportWidge
MonocularReportWidget::MonocularReportWidget( QWidget *parent )
    : ReportWidget( parent )
{
}

void MonocularReportWidget::report( const MonocularCalibrationResults &calibration )
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
    addMatrix( calibration.distorsionCoefficients().t() );
    addBreak();

    addBreak();
    addText( tr( "Reprojection error:" ) +  " " + QString::number( calibration.error() ) +"\n" );
    addBreak();

    for ( auto &i : calibration.results() ) {

        if ( !i.sourceView().empty() ) {

            CvImage undistorted;

            cv::undistort( i.sourceView(), undistorted, calibration.cameraMatrix(), calibration.distorsionCoefficients() );

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

const MonocularCalibrationResults &MonocularReportWidget::calibrationResults() const
{
    return m_calibrationResults;
}

// StereoReportWidget
StereoReportWidget::StereoReportWidget( QWidget *parent )
    : ReportWidget( parent )
{
}

void StereoReportWidget::report(const StereoCalibrationResults &calibration )
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
        addMatrix( calibration.leftCameraResults().distorsionCoefficients().t() );
        addDoubleBreak();

        addText( tr( "Right distorsion coefficients:" ) + " " );
        addMatrix( calibration.rightCameraResults().distorsionCoefficients().t() );
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

        cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

        cv::initUndistortRectifyMap( calibration.leftCameraResults().cameraMatrix(), calibration.leftCameraResults().distorsionCoefficients(),
                                     calibration.leftRectifyMatrix(), calibration.leftProjectionMatrix(), calibration.leftCameraResults().frameSize(),
                                     CV_32FC2, leftRMap, leftDMap );
        cv::initUndistortRectifyMap( calibration.rightCameraResults().cameraMatrix(), calibration.rightCameraResults().distorsionCoefficients(),
                                     calibration.rightRectifyMatrix(), calibration.rightProjectionMatrix(), calibration.rightCameraResults().frameSize(),
                                     CV_32FC2, rightRMap, rightDMap );

        for ( auto i = 0; i < calibration.leftCameraResults().resultsSize(); ++i ) {
            auto leftImage = calibration.leftCameraResults().result(i).sourceView();
            auto rightImage = calibration.rightCameraResults().result(i).sourceView();

            if ( !leftImage.empty() && !rightImage.empty() ) {

                cv::Mat leftRectifiedImage;
                cv::Mat rightRectifiedImage;

                cv::remap( leftImage, leftRectifiedImage, leftRMap, leftDMap, cv::INTER_LANCZOS4 );
                cv::remap( rightImage, rightRectifiedImage, rightRMap, rightDMap, cv::INTER_LANCZOS4 );

                cv::flip( leftRectifiedImage, leftRectifiedImage, -1 );
                cv::flip( rightRectifiedImage, rightRectifiedImage, -1 );

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

const StereoCalibrationResults &StereoReportWidget::calibrationResults() const
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

void MonocularReportDialog::report( const MonocularCalibrationResults &calibration )
{
    m_reportWidget->report( calibration );
}

const MonocularCalibrationResults &MonocularReportDialog::calibrationResults() const
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

void StereoReportDialog::report( const StereoCalibrationResults &calibration )
{
    m_reportWidget->report( calibration );
}

const StereoCalibrationResults &StereoReportDialog::calibrationResults() const
{
    return m_reportWidget->calibrationResults();
}

void StereoReportDialog::initialize()
{
    m_reportWidget = new StereoReportWidget( this );

    m_layout->addWidget( m_reportWidget );

}

