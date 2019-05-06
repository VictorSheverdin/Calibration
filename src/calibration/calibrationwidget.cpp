#include "src/common/precompiled.h"

#include "calibrationwidget.h"

#include "taskwidget.h"
#include "iconswidget.h"
#include "camerawidget.h"
#include "reportwidget.h"
#include "src/common/functions.h"

#include "application.h"
#include "mainwindow.h"

// CalibrationWidgetBase
CalibrationWidgetBase::CalibrationWidgetBase( QWidget *parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void CalibrationWidgetBase::initialize()
{
    m_iconCount = 1;

    m_processor.setAdaptiveThreshold( true );
    m_processor.setFastCheck( false );
    m_processor.setFilterQuads( true );
    m_processor.setNormalizeImage( true );

    m_iconViewDialog = new ImageDialog( application()->mainWindow() );
    m_iconViewDialog->resize( 800, 600 );


}

void CalibrationWidgetBase::setCameraDecimation( VimbaDecimationType type )
{
    m_taskWidget->setCameraDecimation( type );
}

void CalibrationWidgetBase::clearIcons()
{
    m_iconsList->clear();
}

MonocularCalibrationData CalibrationWidgetBase::calcMonocularCalibration(  const std::vector< CvImage > &frames )
{
    MonocularCalibrationData ret;

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

    cv::Mat cameraMatrix = cv::Mat::eye( 3, 3, CV_64F );
    cv::Mat distCoeffs = cv::Mat::zeros( 8, 1, CV_64F );
    std::vector< cv::Mat > rvecs;
    std::vector< cv::Mat > tvecs;

    double rms = cv::calibrateCamera( points3d, points2d, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 );

    bool ok = cv::checkRange( cameraMatrix ) && cv::checkRange( distCoeffs );

    ret.setCameraMatrix( cameraMatrix );
    ret.setDistortionCoefficients( distCoeffs );

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
    m_iconsList = new IconsList( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsList );

    m_taskWidget->resize( 800, 600 );

    m_reportDialog = new MonocularReportDialog( application()->mainWindow() );
    m_reportDialog->resize( 800, 600 );

    // TODO:
    // connect( m_iconsWidget, SIGNAL( iconActivated( IconBase* ) ), this, SLOT( showIcon( IconBase* ) ) );

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

    if ( taskWidget->isTemplateExist() ) {
        m_iconsList->insertIcon( new MonocularIcon( taskWidget->previewImage(), taskWidget->sourceImage(), m_iconCount ) );
        ++m_iconCount;
    }
}

void MonocularCalibrationWidget::calculate()
{
    std::vector< CvImage > frames;

    auto icons = m_iconsList->icons();

    for ( auto &i : icons ) {

        auto image = i->toMonocularIcon()->sourceImage();

        if ( !image.empty() ) {
            frames.push_back( image );
        }

    }

    auto calibrationResult = calcMonocularCalibration( frames );

    m_reportDialog->showMaximized();
    m_reportDialog->activateWindow();

    m_reportDialog->report( calibrationResult );

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
    m_iconsList = new IconsList( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsList );

    m_taskWidget->resize( 800, 600 );

    m_reportDialog = new StereoReportDialog( application()->mainWindow() );
    m_reportDialog->resize( 800, 600 );

    // TODO:
    // connect( m_iconsWidget, SIGNAL( iconActivated( IconBase* ) ), this, SLOT( showIcon( IconBase* ) ) );
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

    if ( taskWidget->isTemplateExist() ) {
        m_iconsList->insertIcon( new StereoIcon(
                                        StereoCameraWidget::makeOverlappedPreview( taskWidget->leftDisplayedImage(), taskWidget->rightDisplayedImage() ),
                                        StereoCameraWidget::makeStraightPreview( taskWidget->leftDisplayedImage(), taskWidget->rightDisplayedImage() ),
                                        taskWidget->leftSourceImage(), taskWidget->rightSourceImage(), m_iconCount ) );

        ++m_iconCount;

    }

}

void StereoCalibrationWidget::calculate()
{
    std::vector< CvImage > leftFrames;
    std::vector< CvImage > rightFrames;

    auto icons = m_iconsList->icons();

    for ( auto i = icons.begin(); i != icons.end(); ++i ) {
        auto leftImage = (*i)->toStereoIcon()->leftSourceImage();
        auto rightImage = (*i)->toStereoIcon()->rightSourceImage();

        if ( !leftImage.empty() && !rightImage.empty() ) {
            leftFrames.push_back( leftImage );
            rightFrames.push_back( rightImage );
        }

    }

    auto calibrationResult = calcStereoCalibration( leftFrames, rightFrames );

    m_reportDialog->showMaximized();
    m_reportDialog->activateWindow();

    m_reportDialog->report( calibrationResult );

}

StereoCalibrationData StereoCalibrationWidget::calcStereoCalibration( const std::vector< CvImage > &leftFrames,
                                               const std::vector< CvImage > &rightFrames )
{
    m_processor.setCount( m_taskWidget->templateCount() );
    m_processor.setSize( m_taskWidget->templateSize() );

    if ( leftFrames.size() != rightFrames.size() )
        throw std::exception();

    if ( leftFrames.size() < m_minimumCalibrationFrames )
        throw std::exception();

    StereoCalibrationData ret;

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

    ret.setCorrespondFrameCount( points3d.size() );

    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    double rms = cv::stereoCalibrate( points3d, leftPoints2d, rightPoints2d,
                                      ret.leftCameraResults().cameraMatrix(), ret.leftCameraResults().distortionCoefficients(),
                                      ret.rightCameraResults().cameraMatrix(), ret.rightCameraResults().distortionCoefficients(), ret.leftCameraResults().frameSize(),
                                      R, T, E, F, cv::CALIB_FIX_INTRINSIC );

    ret.setRotationMatrix( R );
    ret.setTranslationVector( T );
    ret.setFundamentalMatrix( F );
    ret.setEssentialMatrix( E );
    ret.setError( rms );

    cv::Mat R1, R2, P1, P2, Q;

    cv::Rect leftROI;
    cv::Rect rightROI;

    cv::stereoRectify( ret.leftCameraResults().cameraMatrix(), ret.leftCameraResults().distortionCoefficients(),
                       ret.rightCameraResults().cameraMatrix(), ret.rightCameraResults().distortionCoefficients(), ret.leftCameraResults().frameSize(),
                       R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, cv::Size(), &leftROI, &rightROI );


    ret.setLeftRectifyMatrix( R1 );
    ret.setRightRectifyMatrix( R2 );
    ret.setLeftProjectionMatrix( P1 );
    ret.setRightProjectionMatrix( P2 );
    ret.setDisparityToDepthMatrix( Q );
    ret.setLeftROI( leftROI );
    ret.setRightROI( rightROI );

    cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

    cv::initUndistortRectifyMap( ret.leftCameraResults().cameraMatrix(), ret.leftCameraResults().distortionCoefficients(), R1, P1,
                                 ret.leftCameraResults().frameSize(), CV_32FC2, leftRMap, leftDMap );
    cv::initUndistortRectifyMap( ret.rightCameraResults().cameraMatrix(), ret.rightCameraResults().distortionCoefficients(), R2, P2,
                                 ret.leftCameraResults().frameSize(), CV_32FC2, rightRMap, rightDMap );


    ret.setLeftRMap( leftRMap );
    ret.setLeftDMap( leftDMap );
    ret.setRightRMap( rightRMap );
    ret.setRightDMap( rightDMap );

    return ret;
}


