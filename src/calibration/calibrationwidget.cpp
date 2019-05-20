#include "src/common/precompiled.h"

#include "calibrationwidget.h"

#include "taskwidget.h"
#include "iconswidget.h"
#include "camerawidget.h"
#include "reportwidget.h"
#include "parameterswidget.h"
#include "fileslistwidget.h"
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
    m_iconsList = new IconsWidget( this );
    addWidget( m_iconsList );

    connect( m_iconsList, SIGNAL( iconActivated( IconBase* ) ), this, SLOT( showIcon( IconBase* ) ) );

    m_processor.setAdaptiveThreshold( true );
    m_processor.setFastCheck( false );
    m_processor.setFilterQuads( true );
    m_processor.setNormalizeImage( true );

    m_iconViewDialog = new ImageDialog( application()->mainWindow() );
    m_iconViewDialog->resize( 800, 600 );

    dropIconCount();

}

void CalibrationWidgetBase::clearIcons()
{
    m_iconsList->clear();

    dropIconCount();

}

void CalibrationWidgetBase::dropIconCount()
{
    m_iconCount = 1;
}

void CalibrationWidgetBase::addIcon( IconBase *icon )
{
    m_iconsList->addIcon( icon );
    ++m_iconCount;
}

void CalibrationWidgetBase::insertIcon( IconBase *icon )
{
    m_iconsList->insertIcon( icon );
    ++m_iconCount;
}

MonocularCalibrationData CalibrationWidgetBase::calcMonocularCalibration(  const std::vector< CvImage > &frames, const cv::Size &count, const double size )
{
    MonocularCalibrationData ret;

    m_processor.setCount( count );
    m_processor.setSize( size );

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
MonocularCalibrationWidget::MonocularCalibrationWidget( QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize();
}

void MonocularCalibrationWidget::initialize()
{
    m_parametersWidget = new ParametersWidget( this );

    insertWidget( 0, m_parametersWidget );
}

void MonocularCalibrationWidget::importDialog()
{
    auto files = QFileDialog::getOpenFileNames(
                            this,
                            tr( "Select images for calibration" ),
                            QString(),
                            "Image files (*.png *.xpm *.jpg)" );

    for ( auto &i : files )
        loadIcon( i );

}

void MonocularCalibrationWidget::exportDialog()
{

}

void MonocularCalibrationWidget::calculate()
{

}

void MonocularCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->setImage( icon->previewImage() );
    m_iconViewDialog->exec();

}

MonocularIcon *MonocularCalibrationWidget::createIcon( const CvImage &image )
{
    m_processor.setCount( m_parametersWidget->templateCount() );
    m_processor.setSize( m_parametersWidget->templateSize() );

    std::vector< cv::Point2f > points;
    CvImage preview;

    m_processor.processFrame( image, &preview, &points );

    return new MonocularIcon( preview, image, m_iconCount );
}

void MonocularCalibrationWidget::addIcon( const CvImage &image )
{
    CalibrationWidgetBase::addIcon( createIcon( image ) );

}

void MonocularCalibrationWidget::insertIcon( const CvImage &image )
{
    CalibrationWidgetBase::insertIcon( createIcon( image ) );
}

void MonocularCalibrationWidget::loadIcon( const QString &fileName )
{
    CvImage img = cv::imread( fileName.toStdString() );

    if ( !img.empty() ) {
        addIcon( img );

    }

}

// StereoCalibrationWidget
StereoCalibrationWidget::StereoCalibrationWidget( QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize();
}

void StereoCalibrationWidget::initialize()
{
    m_parametersWidget = new ParametersWidget( this );

    insertWidget( 0, m_parametersWidget );

}

void StereoCalibrationWidget::importDialog()
{
    StereoFilesListDialog dlg( this );

    if ( dlg.exec() == StereoFilesListDialog::Accepted ) {
        auto leftFileNames = dlg.leftFileNames();
        auto rightFileNames = dlg.rightFileNames();

        for ( auto i = 0; i < leftFileNames.size(); ++i ) {
            loadIcon( leftFileNames[i], rightFileNames[i] );
        }

    }

}

void StereoCalibrationWidget::exportDialog()
{

}

void StereoCalibrationWidget::calculate()
{

}

void StereoCalibrationWidget::addIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    CalibrationWidgetBase::addIcon( createIcon( leftImage, rightImage ) );
}

void StereoCalibrationWidget::insertIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    CalibrationWidgetBase::insertIcon( createIcon( leftImage, rightImage ) );
}

void StereoCalibrationWidget::loadIcon( const QString &leftFileName, const QString &rightFileName )
{
    CvImage leftImg = cv::imread( leftFileName.toStdString() );
    CvImage rightImg = cv::imread( rightFileName.toStdString() );

    if ( !leftImg.empty() && !rightImg.empty() ) {
        addIcon( leftImg, rightImg );

    }

}

StereoIcon *StereoCalibrationWidget::createIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    m_processor.setCount( m_parametersWidget->templateCount() );
    m_processor.setSize( m_parametersWidget->templateSize() );

    std::vector< cv::Point2f > points;
    CvImage leftPreview;
    CvImage rightPreview;

    m_processor.processFrame( leftImage, &leftPreview, &points );
    m_processor.processFrame( rightImage, &rightPreview, &points );

    return new StereoIcon( leftPreview, rightPreview, leftImage, rightImage, m_iconCount );

}

void StereoCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->toStereoIcon()->straightPreview() );
}

// CameraCalibrationWidgetBase
CameraCalibrationWidgetBase::CameraCalibrationWidgetBase( QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize();
}

void CameraCalibrationWidgetBase::initialize()
{
}

// MonocularCameraCalibrationWidget
MonocularCameraCalibrationWidget::MonocularCameraCalibrationWidget( const QString &cameraIp, QWidget *parent )
    : CameraCalibrationWidgetBase( parent )
{
    initialize( cameraIp );
}

void MonocularCameraCalibrationWidget::initialize( const QString &cameraIp )
{
    m_taskWidget = new MonocularGrabWidget( cameraIp, this );

    insertWidget( 0, m_taskWidget );
    m_taskWidget->resize( 800, 600 );

    m_reportDialog = new MonocularReportDialog( application()->mainWindow() );
    m_reportDialog->resize( 800, 600 );

}

MonocularGrabWidget *MonocularCameraCalibrationWidget::taskWidget() const
{
    return dynamic_cast<MonocularGrabWidget *>( m_taskWidget.data() );
}

void MonocularCameraCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->setImage( icon->previewImage() );
    m_iconViewDialog->exec();

}

void MonocularCameraCalibrationWidget::importDialog()
{

}

void MonocularCameraCalibrationWidget::exportDialog()
{

}

void MonocularCameraCalibrationWidget::grabFrame()
{
    auto taskWidget = this->taskWidget();

    if ( taskWidget->isTemplateExist() ) {
        m_iconsList->insertIcon( new MonocularIcon( taskWidget->previewImage(), taskWidget->sourceImage(), m_iconCount ) );
        ++m_iconCount;
    }
}

void MonocularCameraCalibrationWidget::calculate()
{
    std::vector< CvImage > frames;

    auto icons = m_iconsList->icons();

    for ( auto &i : icons ) {

        auto image = i->toMonocularIcon()->sourceImage();

        if ( !image.empty() ) {
            frames.push_back( image );
        }

    }

    auto calibrationResult = calcMonocularCalibration( frames, m_taskWidget->templateCount(), m_taskWidget->templateSize() );

    m_reportDialog->showMaximized();
    m_reportDialog->activateWindow();

    m_reportDialog->report( calibrationResult );

}

// StereoCameraCalibrationWidget
StereoCameraCalibrationWidget::StereoCameraCalibrationWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent )
    : CameraCalibrationWidgetBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoCameraCalibrationWidget::initialize( const QString &leftCameraIp, const QString &rightCameraIp )
{
    m_taskWidget = new StereoGrabWidget( leftCameraIp, rightCameraIp, this );

    insertWidget( 0, m_taskWidget );

    m_taskWidget->resize( 800, 600 );

    m_reportDialog = new StereoReportDialog( application()->mainWindow() );
    m_reportDialog->resize( 800, 600 );

}

StereoGrabWidget *StereoCameraCalibrationWidget::taskWidget() const
{
    return dynamic_cast< StereoGrabWidget * >( m_taskWidget.data() );
}

void StereoCameraCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->toStereoIcon()->straightPreview() );
}

void StereoCameraCalibrationWidget::importDialog()
{

}

void StereoCameraCalibrationWidget::exportDialog()
{

}

void StereoCameraCalibrationWidget::grabFrame()
{
    auto taskWidget = this->taskWidget();

    if ( taskWidget->isTemplateExist() ) {
        m_iconsList->insertIcon( new StereoIcon( taskWidget->leftDisplayedImage(), taskWidget->rightDisplayedImage(),
                                        taskWidget->leftSourceImage(), taskWidget->rightSourceImage(), m_iconCount ) );

        ++m_iconCount;

    }

}

void StereoCameraCalibrationWidget::calculate()
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

StereoCalibrationData StereoCameraCalibrationWidget::calcStereoCalibration( const std::vector< CvImage > &leftFrames,
                                               const std::vector< CvImage > &rightFrames )
{
    m_processor.setCount( m_taskWidget->templateCount() );
    m_processor.setSize( m_taskWidget->templateSize() );

    if ( leftFrames.size() != rightFrames.size() )
        throw std::exception();

    if ( leftFrames.size() < m_minimumCalibrationFrames )
        throw std::exception();

    StereoCalibrationData ret;

    ret.setLeftCameraResults( calcMonocularCalibration( leftFrames, m_taskWidget->templateCount(), m_taskWidget->templateSize() ) );
    ret.setRightCameraResults( calcMonocularCalibration( rightFrames, m_taskWidget->templateCount(), m_taskWidget->templateSize() ) );

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


