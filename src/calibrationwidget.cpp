#include "precompiled.h"

#include "calibrationwidget.h"

#include "taskwidget.h"
#include "iconswidget.h"
#include "camerawidget.h"
#include "reportwidget.h"
#include "functions.h"

#include "application.h"
#include "mainwindow.h"

// MonocularCalibrationResults
MonocularCalibrationResult::MonocularCalibrationResult()
{
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

// CalibrationWidgetBase
CalibrationWidgetBase::CalibrationWidgetBase( QWidget *parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void CalibrationWidgetBase::initialize()
{
    m_processor.setAdaptiveThreshold( true );
    m_processor.setFastCheck( false );
    m_processor.setFilterQuads( true );
    m_processor.setNormalizeImage( true );

    m_iconViewDialog = new ImageDialog( application()->mainWindow() );
    m_iconViewDialog->resize( 800, 600 );

    m_reportDialog = new ReportDialog( application()->mainWindow() );
    m_reportDialog->resize( 800, 600 );

}

void CalibrationWidgetBase::setCameraDecimation( CameraWidgetBase::DecimationType type )
{
    m_taskWidget->setCameraDecimation( type );
}

MonocularCalibrationResults CalibrationWidgetBase::calcMonocularCalibration(  const std::vector< CvImage > &frames )
{
    MonocularCalibrationResults ret;

    m_processor.setCount( m_taskWidget->templateCount() );
    m_processor.setSize( m_taskWidget->templateSize() );

    cv::Size frameSize;

    // Calculating image size
    for ( auto &i : frames ) {
        auto currentSize = i.size();

        if ( frameSize.empty() )
            frameSize = currentSize;
        else
            if ( frameSize != currentSize ) {
                throw std::exception();
            }

    }

    ret.setFrameSize( frameSize );

    std::vector< MonocularCalibrationResult > results;

    for ( auto i = frames.begin(); i != frames.end(); ++i ) {
        MonocularCalibrationResult result;

        result.setSourceView( *i );
        result.setOk( false );

        if ( !i->empty() ) {
            CvImage view;

            std::vector< cv::Point2f > imagePoints;
            m_processor.processFrame( *i, &view, &imagePoints );

            if ( !imagePoints.empty() ) {

                std::vector< cv::Point3f > objectPoints;
                m_processor.calcChessboardCorners( &objectPoints );

                if ( !objectPoints.empty() ) {

                    result.setProcessedView( view );

                    if ( objectPoints.size() == imagePoints.size() ) {
                        result.setPoints3d( objectPoints );
                        result.setPoints2d( imagePoints );
                        result.setOk( true );
                    }

                }

            }

        }

        results.push_back( result );

    }

    std::vector< std::vector< cv::Point2f > > points2d;
    std::vector< std::vector< cv::Point3f > > points3d;

    for (auto &i : results ) {
        if ( i.isOk() ) {
            points2d.push_back( i.points2d() );
            points3d.push_back( i.points3d() );
        }

    }

    if ( points2d.size() < m_minimumCalibrationFrames || points3d.size() < m_minimumCalibrationFrames || points2d.size() != points3d.size() )
        throw std::exception();

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector< cv::Mat > rvecs;
    std::vector< cv::Mat > tvecs;

    double rms = cv::calibrateCamera( points3d, points2d, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K3/* | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5*/ );

    bool ok = cv::checkRange( cameraMatrix ) && cv::checkRange( distCoeffs );

    ret.setCameraMatrix( cameraMatrix );
    ret.setDistorsionCoefficients( distCoeffs );

    int index = 0;
    for (auto &i : results ) {
        if (i.isOk() ) {
            i.setRVec( rvecs[index] );
            i.setTVec( tvecs[index] );
        }

    }

    ret.setResults( results );
    ret.setError( rms );
    ret.setOk( ok );

    return ret;

}

// MonocularCalibrationWidget
MonocularCalibrationWidget::MonocularCalibrationWidget( const std::string &cameraIp, QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize( cameraIp );
}

void MonocularCalibrationWidget::initialize(const std::string &cameraIp )
{
    m_taskWidget = new MonocularTaskWidget( cameraIp, this );
    m_iconsWidget = new IconsWidget( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsWidget );

    connect( m_iconsWidget, SIGNAL( iconActivated( IconBase* ) ), this, SLOT( showIcon( IconBase* ) ) );

}

MonocularTaskWidget *MonocularCalibrationWidget::taskWidget() const
{
    return dynamic_cast<MonocularTaskWidget *>( m_taskWidget.data() );
}

void MonocularCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->previewImage() );

}

void MonocularCalibrationWidget::grabFrame()
{
    auto taskWidget = this->taskWidget();

    if ( taskWidget->isTemplateExist() )
        m_iconsWidget->insertIcon( new MonocularIcon( taskWidget->previewImage(), taskWidget->sourceImage(), this ) );
}

void MonocularCalibrationWidget::calculate()
{
    std::vector< CvImage > frames;

    auto icons = m_iconsWidget->icons();

    for ( auto &i : icons ) {

        auto image = i->toMonocularIcon()->sourceImage();

        if ( !image.empty() ) {
            frames.push_back( image );
        }

    }

    auto calibrationResult = calcMonocularCalibration( frames );

    m_reportDialog->showMaximized();
    m_reportDialog->activateWindow();

    m_reportDialog->clear();

    m_reportDialog->addText( tr( "Camera calibration." ) + "\n" );
    m_reportDialog->addBreak();


    for (  auto &i : calibrationResult.results() ) {

        if ( !i.processedView().empty() ) {

            CvImage resized;

            cv::resize( i.processedView(), resized, cv::Size(), 0.25, 0.25, cv::INTER_LANCZOS4 );
            m_reportDialog->addImage( resized );

        }

    }

    m_reportDialog->addDoubleBreak();

    m_reportDialog->addText( tr( "Results:" ) + "\n" );
    m_reportDialog->addBreak();

    m_reportDialog->addText( tr( "Camera matrix:" ) + "\n" );
    m_reportDialog->addMatrix( calibrationResult.cameraMatrix() );
    m_reportDialog->addBreak();

    m_reportDialog->addBreak();
    m_reportDialog->addText( tr( "Distorsion coefficients:" ) + " " );
    m_reportDialog->addMatrix( calibrationResult.distorsionCoefficients().t() );
    m_reportDialog->addBreak();

    m_reportDialog->addBreak();
    m_reportDialog->addText( tr( "Reprojection error:" ) +  " " + QString::number( calibrationResult.error() ) +"\n" );
    m_reportDialog->addBreak();

    for ( auto &i : calibrationResult.results() ) {

        if ( !i.sourceView().empty() ) {

            CvImage undistorted;

            cv::undistort( i.sourceView(), undistorted, calibrationResult.cameraMatrix(), calibrationResult.distorsionCoefficients() );

            CvImage resizedView;
            cv::resize( undistorted, resizedView, cv::Size(), 0.25, 0.25, cv::INTER_LANCZOS4 );

            m_reportDialog->addImage( resizedView );

        }

    }

    m_reportDialog->addDoubleBreak();

    if ( calibrationResult.isOk() )
        m_reportDialog->addText( tr( "Calibration succesful!" ) + "\n" );
    else
        m_reportDialog->addText( tr( "Calibration error!" ) + "\n" );



}

// StereoCalibrationWidget
StereoCalibrationWidget::StereoCalibrationWidget(const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoCalibrationWidget::initialize( const std::string &leftCameraIp, const std::string &rightCameraIp )
{
    m_taskWidget = new StereoTaskWidget( leftCameraIp, rightCameraIp, this );
    m_iconsWidget = new IconsWidget( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsWidget );

    connect( m_iconsWidget, SIGNAL( iconActivated( IconBase* ) ), this, SLOT( showIcon( IconBase* ) ) );
}

void StereoCalibrationWidget::saveXMLCalibration()
{

}

StereoTaskWidget *StereoCalibrationWidget::taskWidget() const
{
    return dynamic_cast<StereoTaskWidget *>( m_taskWidget.data() );
}

void StereoCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->toStereoIcon()->straightPreview() );

}

void StereoCalibrationWidget::grabFrame()
{
    auto taskWidget = this->taskWidget();

    if ( taskWidget->isTemplateExist() )
        m_iconsWidget->insertIcon( new StereoIcon(
                                        StereoCameraWidget::makeOverlappedPreview( taskWidget->leftDisplayedImage(), taskWidget->rightDisplayedImage() ),
                                        StereoCameraWidget::makeStraightPreview( taskWidget->leftDisplayedImage(), taskWidget->rightDisplayedImage() ),
                                        taskWidget->leftSourceImage(), taskWidget->rightSourceImage(), this ) );
}

void StereoCalibrationWidget::calculate()
{
    m_processor.setCount( m_taskWidget->templateCount() );
    m_processor.setSize( m_taskWidget->templateSize() );

    m_reportDialog->showMaximized();
    m_reportDialog->activateWindow();

    m_reportDialog->clear();

    m_reportDialog->addText( tr( "Stereo camera calibration." ) + "\n" );
    m_reportDialog->addBreak();

    std::vector< CvImage > leftFrames;
    std::vector< CvImage > rightFrames;

    auto icons = m_iconsWidget->icons();

    for ( auto i = icons.begin(); i != icons.end(); ++i ) {
        auto leftImage = (*i)->toStereoIcon()->leftSourceImage();
        auto rightImage = (*i)->toStereoIcon()->rightSourceImage();

        if ( !leftImage.empty() && !rightImage.empty() ) {
            leftFrames.push_back( leftImage );
            rightFrames.push_back( rightImage );
        }

    }

    auto calibrationResult = calcStereoCalibration( leftFrames, rightFrames );

    if ( calibrationResult.leftResultsSize() != calibrationResult.rightResultsSize() )
        throw std::exception();

        for (  int i = 0; i < calibrationResult.leftCameraResults().resultsSize(); ++i ) {
            if ( calibrationResult.leftCameraResults().result(i).isOk() &&  calibrationResult.rightCameraResults().result(i).isOk() ) {

                auto image1 = resizeTo( calibrationResult.leftCameraResults().result( i ).processedView(), m_reportFrameSize );
                auto image2 = resizeTo( calibrationResult.rightCameraResults().result( i ).processedView(), m_reportFrameSize );

                auto stitchedImage = StereoCameraWidget::makeStraightPreview( image1, image2 );

                if ( !stitchedImage.empty() ) {
                    m_reportDialog->addImage( stitchedImage );
                    m_reportDialog->addSpace();
                }

            }

        }

        m_reportDialog->addDoubleBreak();

        if ( calibrationResult.isOk() ) {

            m_reportDialog->addText( tr( "Left camera matrix:" ) + "\n" );
            m_reportDialog->addMatrix( calibrationResult.leftCameraResults().cameraMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right camera matrix:" ) + "\n" );
            m_reportDialog->addMatrix( calibrationResult.rightCameraResults().cameraMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Left distorsion coefficients:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.leftCameraResults().distorsionCoefficients().t() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right distorsion coefficients:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.rightCameraResults().distorsionCoefficients().t() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Rotation matrix:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.rotationMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Translation vector:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.translationVector().t() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Fundamental matrix:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.fundamentalMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Essential matrix:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.essentialMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Left camera rectification matrix:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.leftRectifyMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Left camera projection matrix:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.leftProjectionMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right camera rectification matrix:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.rightRectifyMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right camera projection matrix:" ) + " " );
            m_reportDialog->addMatrix( calibrationResult.rightProjectionMatrix() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Left valid ROI:" ) + " " );
            m_reportDialog->addRect( calibrationResult.leftROI() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right valid ROI:" ) + " " );
            m_reportDialog->addRect( calibrationResult.rightROI() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Reprojection error:" ) +  " " + QString::number( calibrationResult.error() ) +"\n" );
            m_reportDialog->addBreak();

            cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

            cv::initUndistortRectifyMap( calibrationResult.leftCameraResults().cameraMatrix(), calibrationResult.leftCameraResults().distorsionCoefficients(),
                                         calibrationResult.leftRectifyMatrix(), calibrationResult.leftProjectionMatrix(), calibrationResult.leftCameraResults().frameSize(),
                                         CV_32FC2, leftRMap, leftDMap );
            cv::initUndistortRectifyMap( calibrationResult.rightCameraResults().cameraMatrix(), calibrationResult.rightCameraResults().distorsionCoefficients(),
                                         calibrationResult.rightRectifyMatrix(), calibrationResult.rightProjectionMatrix(), calibrationResult.rightCameraResults().frameSize(),
                                         CV_32FC2, rightRMap, rightDMap );

            for ( auto i = 0; i < calibrationResult.leftCameraResults().resultsSize(); ++i ) {
                auto leftImage = calibrationResult.leftCameraResults().result(i).sourceView();
                auto rightImage = calibrationResult.rightCameraResults().result(i).sourceView();

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

                        m_reportDialog->addImage( stitchedImage );
                        m_reportDialog->addSpace();

                    }


                }

            }

            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Calibration succesful!" ) + "\n" );

        }
        else
            m_reportDialog->addText( tr( "Calibration error!" ) + "\n" );

}

StereoCalibrationResults StereoCalibrationWidget::calcStereoCalibration( const std::vector< CvImage > &leftFrames,
                                               const std::vector< CvImage > &rightFrames )
{
    if ( leftFrames.size() != rightFrames.size() )
        throw std::exception();

    if ( leftFrames.size() < m_minimumCalibrationFrames )
        throw std::exception();

    StereoCalibrationResults ret;

    ret.setLeftCameraResults( calcMonocularCalibration( leftFrames ) );
    ret.setRightCameraResults( calcMonocularCalibration( rightFrames ) );

    if ( ret.leftCameraResults().frameSize() != ret.rightCameraResults().frameSize() )
        throw std::exception();

    std::vector< std::vector< cv::Point2f > > leftPoints2d;
    std::vector< std::vector< cv::Point2f > > rightPoints2d;
    std::vector< std::vector< cv::Point3f > > points3d;

    if ( ret.leftCameraResults().resultsSize() != ret.rightCameraResults().resultsSize() )
        throw std::exception();

    for ( auto i = 0; i < leftFrames.size(); ++i )
        if ( ret.leftCameraResults().result(i).isOk() && ret.rightCameraResults().result(i).isOk() ) {
            points3d.push_back( ret.leftCameraResults().result(i).points3d() );
            leftPoints2d.push_back( ret.leftCameraResults().result(i).points2d() );
            rightPoints2d.push_back( ret.rightCameraResults().result(i).points2d() );
        }

    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    double rms = cv::stereoCalibrate( points3d, leftPoints2d, rightPoints2d,
                                      ret.leftCameraResults().cameraMatrix(), ret.leftCameraResults().distorsionCoefficients(),
                                      ret.rightCameraResults().cameraMatrix(), ret.rightCameraResults().distorsionCoefficients(), ret.leftCameraResults().frameSize(),
                                      R, T, E, F, cv::CALIB_FIX_INTRINSIC );

    ret.setRotationMatrix( R );
    ret.setTranslationVector( T );
    ret.setFundamentalMatrix( F );
    ret.setEssentialMatrix( E );
    ret.setError( rms );

    cv::Mat R1, R2, P1, P2, Q;

    cv::Rect leftROI;
    cv::Rect rightROI;

    cv::stereoRectify( ret.leftCameraResults().cameraMatrix(), ret.leftCameraResults().distorsionCoefficients(),
                       ret.rightCameraResults().cameraMatrix(), ret.rightCameraResults().distorsionCoefficients(), ret.leftCameraResults().frameSize(),
                       R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, cv::Size(), &leftROI, &rightROI );


    ret.setLeftRectifyMatrix( R1 );
    ret.setRightRectifyMatrix( R2 );
    ret.setLeftProjectionMatrix( P1 );
    ret.setRightProjectionMatrix( P2 );
    ret.setDisparityToDepthMatrix( Q );
    ret.setLeftROI( leftROI );
    ret.setRightROI( rightROI );

    cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

    cv::initUndistortRectifyMap( ret.leftCameraResults().cameraMatrix(), ret.leftCameraResults().distorsionCoefficients(), R1, P1,
                                 ret.leftCameraResults().frameSize(), CV_16SC2, leftRMap, leftDMap );
    cv::initUndistortRectifyMap( ret.rightCameraResults().cameraMatrix(), ret.rightCameraResults().distorsionCoefficients(), R2, P2,
                                 ret.leftCameraResults().frameSize(), CV_16SC2, rightRMap, rightDMap );


    ret.setLeftRMap( leftRMap );
    ret.setLeftDMap( leftDMap );
    ret.setRightRMap( rightRMap );
    ret.setRightDMap( rightDMap );

    return ret;
}


