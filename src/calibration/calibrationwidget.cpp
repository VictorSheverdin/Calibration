#include "src/common/precompiled.h"

#include "calibrationwidget.h"

#include "taskwidget.h"
#include "calibrationiconswidget.h"
#include "camerawidget.h"
#include "reportwidget.h"
#include "parameterswidget.h"
#include "src/common/fileslistwidget.h"
#include "src/common/functions.h"

#include "application.h"
#include "mainwindow.h"

#include "documentwidget.h"

// CalibrationWidgetBase
CalibrationWidgetBase::CalibrationWidgetBase( QWidget *parent )
    : QWidget( parent )
{
    initialize();
}

void CalibrationWidgetBase::initialize()
{
    setAttribute( Qt::WA_DeleteOnClose );

    m_layout = new QVBoxLayout( this );

    m_iconsList = new CalibrationIconsWidget( this );

    connect( m_iconsList, SIGNAL( iconActivated( CalibrationIconBase* ) ), this, SLOT( showIcon( CalibrationIconBase* ) ) );

    m_templateProcessor.setAdaptiveThreshold( true );
    m_templateProcessor.setFastCheck( false );
    m_templateProcessor.setFilterQuads( true );
    m_templateProcessor.setNormalizeImage( true );

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

void CalibrationWidgetBase::addIcon( CalibrationIconBase *icon )
{
    m_iconsList->addIcon( icon );
}

void CalibrationWidgetBase::insertIcon( CalibrationIconBase *icon )
{
    m_iconsList->insertIcon( icon );
}

MonocularCalibrationData CalibrationWidgetBase::calcMonocularCalibration( const QList< CalibrationIconBase *> &icons, const cv::Size &count, const double size )
{
    MonocularCalibrationData ret;

    if ( !icons.empty() ) {

        m_templateProcessor.setCount( count );
        m_templateProcessor.setSize( size );

        std::vector< cv::Point3f > objectPoints;
        m_templateProcessor.calcCorners( &objectPoints );

        if ( !objectPoints.empty() ) {

            cv::Size frameSize;

            std::vector< MonocularCalibrationResult > results;

            std::vector< std::vector< cv::Point2f > > points2d;
            std::vector< std::vector< cv::Point3f > > points3d;

            for ( auto &i : icons ) {

                auto currentIcon = i->toMonocularIcon();

                if ( currentIcon ) {

                    auto currentPoints = currentIcon->points();

                    if ( !currentPoints.empty() ) {

                        if ( !objectPoints.empty() ) {
                            points2d.push_back( currentPoints );
                            points3d.push_back( objectPoints );

                            MonocularCalibrationResult result;
                            result.setOk( true );
                            results.push_back( result );

                        }

                    }

                    if ( frameSize.empty() )
                        frameSize = currentIcon->frameSize();
                    else if ( frameSize != currentIcon->frameSize() )
                        throw std::exception();

                }

            }

            ret.setFrameSize( frameSize );

            ret.setPreviewImage( icons.front()->previewImage() );

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
                    ++index;
                }

            }

            ret.setResults( results );
            ret.setError( rms );
            ret.setOk( ok );

        }

    }

    return ret;

}

MonocularCalibrationData CalibrationWidgetBase::calcMonocularCalibration( const std::vector< std::vector< cv::Point2f > > &points, cv::Size &frameSize, const cv::Size &count, const double size )
{
    MonocularCalibrationData ret;

    m_templateProcessor.setCount( count );
    m_templateProcessor.setSize( size );

    ret.setFrameSize( frameSize );

    std::vector< MonocularCalibrationResult > results;

    std::vector< std::vector< cv::Point2f > > points2d;
    std::vector< std::vector< cv::Point3f > > points3d;

    for ( auto i = points.begin(); i != points.end(); ++i ) {
        if ( !i->empty() ) {

            std::vector< cv::Point3f > objectPoints;
            m_templateProcessor.calcCorners( &objectPoints );

            if ( !objectPoints.empty() ) {

                if ( objectPoints.size() == i->size() ) {
                    MonocularCalibrationResult result;
                    result.setOk( true );
                    results.push_back( result );

                    points3d.push_back( objectPoints );
                    points2d.push_back( *i );

                }

            }

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
            ++index;
        }

    }

    ret.setResults( results );
    ret.setError( rms );
    ret.setOk( ok );

    return ret;

}

StereoCalibrationData CalibrationWidgetBase::calcStereoCalibration(const QList< CalibrationIconBase * > &icons, const cv::Size &count, const double size )
{
    StereoCalibrationData ret;

    m_templateProcessor.setCount( count );
    m_templateProcessor.setSize( size );

    std::vector< cv::Point3f > objectPoints;
    m_templateProcessor.calcCorners( &objectPoints );

    if ( !objectPoints.empty() ) {

        std::vector< std::vector< cv::Point3f > > points3d;
        std::vector< std::vector< cv::Point2f > > leftPoints;
        std::vector< std::vector< cv::Point2f > > rightPoints;

        cv::Size frameSize;

        for ( auto i = icons.begin(); i != icons.end(); ++i ) {

            auto currentIcon = (*i)->toStereoIcon();

            auto currentLeftPoints = currentIcon->leftPoints();
            auto currentRightPoints = currentIcon->rightPoints();

            if ( currentLeftPoints.size() == currentRightPoints.size() && currentLeftPoints.size() == objectPoints.size() ) {
                points3d.push_back( objectPoints );
                leftPoints.push_back( currentLeftPoints );
                rightPoints.push_back( currentRightPoints );
            }

            if ( frameSize.empty() )
                frameSize = currentIcon->frameSize();
            else if ( frameSize != currentIcon->frameSize() )
                throw std::exception();

        }

        if ( leftPoints.size() != rightPoints.size() )
            throw std::exception();

        if ( leftPoints.size() < m_minimumCalibrationFrames )
            throw std::exception();

        ret.setLeftCameraResults( calcMonocularCalibration( leftPoints, frameSize, count, size ) );
        ret.setRightCameraResults( calcMonocularCalibration( rightPoints, frameSize, count, size ) );

        ret.leftCameraResults().setPreviewImage( icons.front()->toStereoIcon()->leftPreview() );
        ret.rightCameraResults().setPreviewImage( icons.front()->toStereoIcon()->rightPreview() );

        ret.setCorrespondFrameCount( points3d.size() );

        cv::Mat R;
        cv::Mat T;
        cv::Mat E;
        cv::Mat F;

        double rms = cv::stereoCalibrate( points3d, leftPoints, rightPoints,
                                          ret.leftCameraResults().cameraMatrix(), ret.leftCameraResults().distortionCoefficients(),
                                          ret.rightCameraResults().cameraMatrix(), ret.rightCameraResults().distortionCoefficients(), ret.leftCameraResults().frameSize(),
                                          R, T, E, F, /*cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | */cv::CALIB_USE_INTRINSIC_GUESS/* | cv::CALIB_SAME_FOCAL_LENGTH |
                                          cv::CALIB_RATIONAL_MODEL */| cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5,
                                          cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5 ) );

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

        ret.setOk( true );

    }

    return ret;

}

// MonocularCalibrationWidgetBase
MonocularCalibrationWidgetBase::MonocularCalibrationWidgetBase( QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize();
}

void MonocularCalibrationWidgetBase::initialize()
{
}

void MonocularCalibrationWidgetBase::showIcon( CalibrationIconBase *icon )
{
    m_iconViewDialog->setImage( icon->previewImage() );
    m_iconViewDialog->exec();

}


// StereoCalibrationWidgetBase
StereoCalibrationWidgetBase::StereoCalibrationWidgetBase( QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize();
}

void StereoCalibrationWidgetBase::initialize()
{
}

void StereoCalibrationWidgetBase::showIcon( CalibrationIconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->toStereoIcon()->straightPreview() );
}

// MonocularImageCalibrationWidget
MonocularImageCalibrationWidget::MonocularImageCalibrationWidget( QWidget *parent )
    : MonocularCalibrationWidgetBase( parent )
{
    initialize();
}

void MonocularImageCalibrationWidget::initialize()
{
    m_parametersWidget = new ParametersWidget( this );

    m_layout->addWidget( m_parametersWidget );
    m_layout->addWidget( m_iconsList );

}

void MonocularImageCalibrationWidget::importDialog()
{
    auto files = QFileDialog::getOpenFileNames(
                            this,
                            tr( "Select images for calibration" ),
                            QString(),
                            "Image files (*.png *.xpm *.jpg)" );

    for ( auto &i : files )
        loadIcon( i );

}

void MonocularImageCalibrationWidget::exportDialog()
{

}

void MonocularImageCalibrationWidget::calculate()
{
    auto calibrationResult = calcMonocularCalibration( m_iconsList->icons(), m_parametersWidget->templateCount(), m_parametersWidget->templateSize() );

    auto doc = application()->mainWindow()->addMonocularReportDocument();

    doc->report( calibrationResult );

}

MonocularIcon *MonocularImageCalibrationWidget::createIcon( const CvImage &image )
{
    m_templateProcessor.setCount( m_parametersWidget->templateCount() );
    m_templateProcessor.setSize( m_parametersWidget->templateSize() );

    std::vector< cv::Point2f > points;
    CvImage preview;

    m_templateProcessor.processFrame( image, &preview, &points );

    return new MonocularIcon( preview, image.size(), points, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) );
}

void MonocularImageCalibrationWidget::addIcon( const CvImage &image )
{
    CalibrationWidgetBase::addIcon( createIcon( image ) );

}

void MonocularImageCalibrationWidget::insertIcon( const CvImage &image )
{
    CalibrationWidgetBase::insertIcon( createIcon( image ) );
}

void MonocularImageCalibrationWidget::loadIcon( const QString &fileName )
{
    CvImage img = cv::imread( fileName.toStdString() );

    if ( !img.empty() ) {
        addIcon( img );

    }

}

// StereoImageCalibrationWidget
StereoImageCalibrationWidget::StereoImageCalibrationWidget( QWidget *parent )
    : StereoCalibrationWidgetBase( parent )
{
    initialize();
}

void StereoImageCalibrationWidget::initialize()
{
    m_parametersWidget = new ParametersWidget( this );

    m_layout->addWidget( m_parametersWidget );
    m_layout->addWidget( m_iconsList );

}

void StereoImageCalibrationWidget::importDialog()
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

void StereoImageCalibrationWidget::exportDialog()
{

}

void StereoImageCalibrationWidget::calculate()
{
    auto calibrationResult = calcStereoCalibration( m_iconsList->icons(), m_parametersWidget->templateCount(), m_parametersWidget->templateSize() );

    auto doc = application()->mainWindow()->addStereoReportDocument();

    doc->report( calibrationResult );

}

void StereoImageCalibrationWidget::addIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    CalibrationWidgetBase::addIcon( createIcon( leftImage, rightImage ) );
}

void StereoImageCalibrationWidget::insertIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    CalibrationWidgetBase::insertIcon( createIcon( leftImage, rightImage ) );
}

void StereoImageCalibrationWidget::loadIcon( const QString &leftFileName, const QString &rightFileName )
{
    CvImage leftImg = cv::imread( leftFileName.toStdString() );
    CvImage rightImg = cv::imread( rightFileName.toStdString() );

    if ( !leftImg.empty() && !rightImg.empty() ) {
        addIcon( leftImg, rightImg );

    }

}

StereoIcon *StereoImageCalibrationWidget::createIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    m_templateProcessor.setCount( m_parametersWidget->templateCount() );
    m_templateProcessor.setSize( m_parametersWidget->templateSize() );

    if ( leftImage.size() == rightImage.size() ) {

        std::vector< cv::Point2f > leftPoints;
        std::vector< cv::Point2f > rightPoints;
        CvImage leftPreview;
        CvImage rightPreview;

        m_templateProcessor.processFrame( leftImage, &leftPreview, &leftPoints );
        m_templateProcessor.processFrame( rightImage, &rightPreview, &rightPoints );

        return new StereoIcon( leftPreview, rightPreview,
                               leftImage.size(), leftPoints, rightPoints,
                               QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) );

    }

    return nullptr;

}

// CameraCalibrationWidgetBase
CameraCalibrationWidgetBase::CameraCalibrationWidgetBase()
{
    initialize();
}

void CameraCalibrationWidgetBase::initialize()
{
}

// MonocularCameraCalibrationWidget
MonocularCameraCalibrationWidget::MonocularCameraCalibrationWidget( const QString &cameraIp, QWidget *parent )
    : MonocularCalibrationWidgetBase( parent )
{
    initialize( cameraIp );
}

void MonocularCameraCalibrationWidget::initialize( const QString &cameraIp )
{
    m_splitter = new QSplitter( Qt::Vertical, this );

    m_layout->addWidget( m_splitter );

    m_taskWidget = new MonocularGrabWidget( cameraIp, this );
    m_taskWidget->resize( 800, 600 );

    m_splitter->addWidget( m_taskWidget );
    m_splitter->addWidget( m_iconsList );

}

MonocularGrabWidget *MonocularCameraCalibrationWidget::taskWidget() const
{
    return dynamic_cast<MonocularGrabWidget *>( m_taskWidget.data() );
}

void MonocularCameraCalibrationWidget::importDialog()
{
    auto files = QFileDialog::getOpenFileNames(
                            this,
                            tr( "Select images for calibration" ),
                            QString(),
                            "Image files (*.png *.xpm *.jpg)" );

    for ( auto &i : files )
        loadIcon( i );

}

void MonocularCameraCalibrationWidget::exportDialog()
{

}

void MonocularCameraCalibrationWidget::calculate()
{
    auto calibrationResult = calcMonocularCalibration( m_iconsList->icons(), m_taskWidget->templateCount(), m_taskWidget->templateSize() );

    auto doc = application()->mainWindow()->addMonocularReportDocument();

    doc->report( calibrationResult );

}

void MonocularCameraCalibrationWidget::grabFrame()
{
    auto taskWidget = this->taskWidget();

    if ( taskWidget->isTemplateExist() ) {
        m_iconsList->insertIcon( createIcon( taskWidget->sourceImage() ) );
    }
}

MonocularIcon *MonocularCameraCalibrationWidget::createIcon( const CvImage &image )
{
    m_templateProcessor.setCount( m_taskWidget->templateCount() );
    m_templateProcessor.setSize( m_taskWidget->templateSize() );

    std::vector< cv::Point2f > points;
    CvImage preview;

    m_templateProcessor.processFrame( image, &preview, &points );

    return new MonocularIcon( preview, image.size(), points, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) );
}

void MonocularCameraCalibrationWidget::addIcon( const CvImage &image )
{
    CalibrationWidgetBase::addIcon( createIcon( image ) );
}

void MonocularCameraCalibrationWidget::insertIcon( const CvImage &image )
{
    CalibrationWidgetBase::insertIcon( createIcon( image ) );
}

void MonocularCameraCalibrationWidget::loadIcon( const QString &fileName )
{
    CvImage img = cv::imread( fileName.toStdString() );

    if ( !img.empty() ) {
        addIcon( img );

    }

}

// StereoCameraCalibrationWidget
StereoCameraCalibrationWidget::StereoCameraCalibrationWidget( const QString &leftCameraIp, const QString &rightCameraIp, QWidget *parent )
    : StereoCalibrationWidgetBase( parent )
{
    initialize( leftCameraIp, rightCameraIp );
}

void StereoCameraCalibrationWidget::initialize( const QString &leftCameraIp, const QString &rightCameraIp )
{
    m_splitter = new QSplitter( Qt::Vertical, this );

    m_layout->addWidget( m_splitter );

    m_taskWidget = new StereoGrabWidget( leftCameraIp, rightCameraIp, this );
    m_taskWidget->resize( 800, 600 );

    m_splitter->addWidget( m_taskWidget );
    m_splitter->addWidget( m_iconsList );

}

StereoGrabWidget *StereoCameraCalibrationWidget::taskWidget() const
{
    return dynamic_cast< StereoGrabWidget * >( m_taskWidget.data() );
}

void StereoCameraCalibrationWidget::importDialog()
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

void StereoCameraCalibrationWidget::exportDialog()
{

}

void StereoCameraCalibrationWidget::grabFrame()
{
    auto taskWidget = this->taskWidget();

    if ( taskWidget->isTemplateExist() ) {

        auto frame = taskWidget->frame();

        if ( !frame.empty() )
            insertIcon( frame.leftFrame(), frame.rightFrame() );

    }

}

void StereoCameraCalibrationWidget::calculate()
{
    auto calibrationResult = calcStereoCalibration( m_iconsList->icons(), m_taskWidget->templateCount(), m_taskWidget->templateSize() );

    auto doc = application()->mainWindow()->addStereoReportDocument();

    doc->report( calibrationResult );

}

void StereoCameraCalibrationWidget::addIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    CalibrationWidgetBase::addIcon( createIcon( leftImage, rightImage ) );
}

void StereoCameraCalibrationWidget::insertIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    CalibrationWidgetBase::insertIcon( createIcon( leftImage, rightImage ) );
}

void StereoCameraCalibrationWidget::loadIcon( const QString &leftFileName, const QString &rightFileName )
{
    CvImage leftImg = cv::imread( leftFileName.toStdString() );
    CvImage rightImg = cv::imread( rightFileName.toStdString() );

    if ( !leftImg.empty() && !rightImg.empty() ) {
        addIcon( leftImg, rightImg );

    }

}

StereoIcon *StereoCameraCalibrationWidget::createIcon( const CvImage &leftImage, const CvImage &rightImage )
{
    m_templateProcessor.setCount( m_taskWidget->templateCount() );
    m_templateProcessor.setSize( m_taskWidget->templateSize() );

    if ( leftImage.size() == rightImage.size() ) {

        std::vector< cv::Point2f > leftPoints;
        std::vector< cv::Point2f > rightPoints;
        CvImage leftPreview;
        CvImage rightPreview;

        m_templateProcessor.processFrame( leftImage, &leftPreview, &leftPoints );
        m_templateProcessor.processFrame( rightImage, &rightPreview, &rightPoints );

        return new StereoIcon( leftPreview, rightPreview,
                               leftImage.size(), leftPoints, rightPoints,
                               QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) );

    }

    return nullptr;

}

