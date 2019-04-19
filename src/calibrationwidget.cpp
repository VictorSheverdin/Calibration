#include "precompiled.h"

#include "calibrationwidget.h"

#include "taskwidget.h"
#include "iconswidget.h"
#include "camerawidget.h"
#include "reportwidget.h"

#include "application.h"
#include "mainwindow.h"

// MonocularCalibrationResult
MonocularCalibrationResult::MonocularCalibrationResult()
{
}

void MonocularCalibrationResult::setFrameSize( const cv::Size &value )
{
    m_frameSize = value;
}

const cv::Size &MonocularCalibrationResult::frameSize() const
{
    return m_frameSize;
}

void MonocularCalibrationResult::setViews( std::vector< CvImage > &value )
{
    m_views = value;
}

const std::vector<CvImage> &MonocularCalibrationResult::views() const
{
    return m_views;
}

const CvImage &MonocularCalibrationResult::view( const unsigned int i ) const
{
    return m_views[i];
}

void MonocularCalibrationResult::setCameraMatrix( const cv::Mat &value )
{
    m_cameraMatrix = value;
}

const cv::Mat &MonocularCalibrationResult::cameraMatrix() const
{
    return m_cameraMatrix;
}

void MonocularCalibrationResult::setDistorsionCoefficients( const cv::Mat &value )
{
    m_distCoefficients = value;
}

const cv::Mat &MonocularCalibrationResult::distorsionCoefficients() const
{
    return m_distCoefficients;
}

void MonocularCalibrationResult::setRVecs( const std::vector< cv::Mat > &value )
{
    m_rVecs = value;
}

const std::vector< cv::Mat > &MonocularCalibrationResult::rVecs() const
{
    return m_rVecs;
}

const cv::Mat &MonocularCalibrationResult::rVec( const unsigned int i ) const
{
    return m_rVecs[i];
}

void MonocularCalibrationResult::setTVecs( const std::vector< cv::Mat > &value )
{
    m_tVecs = value;
}

const std::vector< cv::Mat > &MonocularCalibrationResult::tVecs() const
{
    return m_tVecs;
}

const cv::Mat &MonocularCalibrationResult::tVec( const unsigned int i ) const
{
    return m_tVecs[i];
}

void MonocularCalibrationResult::setOk( const bool value )
{
    m_ok = value;
}

bool MonocularCalibrationResult::isOk() const
{
    return m_ok;
}

void MonocularCalibrationResult::setError( const double value )
{
    m_error = value;
}

double MonocularCalibrationResult::error() const
{
    return m_error;
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

MonocularCalibrationResult CalibrationWidgetBase::calcMonocularCalibration(  const std::vector< CvImage > &frames )
{
    MonocularCalibrationResult ret;

    m_processor.setCount( m_taskWidget->templateCount() );
    m_processor.setSize( m_taskWidget->templateSize() );

    std::vector< std::vector< cv::Point2f > > points2d;
    std::vector< std::vector< cv::Point3f > > points3d;

    std::vector< CvImage > validFrames;

    for ( auto i = frames.begin(); i != frames.end(); ++i ) {

        if ( !i->empty() ) {
            CvImage view;

            std::vector< cv::Point2f > imagePoints;
            m_processor.processFrame( *i, &view, &imagePoints );

            if ( !imagePoints.empty() ) {

                std::vector< cv::Point3f > objectPoints;
                m_processor.calcChessboardCorners( &objectPoints );

                if ( !objectPoints.empty() ) {
                    validFrames.push_back( view );
                    points2d.push_back( imagePoints );
                    points3d.push_back( objectPoints );
                }

            }

        }

    }

    cv::Size frameSize;

    // Calculating image size
    for ( auto &i : validFrames ) {
        auto currentSize = i.size();

        if ( frameSize.empty() )
            frameSize = currentSize;
        else
            if ( frameSize != currentSize ) {
                throw std::exception();
            }

    }

    ret.setFrameSize( frameSize );

    ret.setViews( validFrames );

    if ( points2d.size() > 0 && points3d.size() > 0 &&
         points2d.size() == points3d.size() &&
         points2d.size() == validFrames.size() ) {

        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_32F);
        std::vector< cv::Mat > rvecs;
        std::vector< cv::Mat > tvecs;

        double rms = cv::calibrateCamera( points3d, points2d, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs );

        bool ok = cv::checkRange( cameraMatrix ) && cv::checkRange( distCoeffs );


        ret.setCameraMatrix( cameraMatrix );
        ret.setDistorsionCoefficients( distCoeffs );

        ret.setRVecs( rvecs );
        ret.setTVecs( tvecs );

        ret.setError( rms );
        ret.setOk( ok );

    }
    else
        throw std::exception();

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


    for (  auto &i : calibrationResult.views() ) {

        if ( !i.empty() ) {

            CvImage resized;

            cv::resize( i, resized, cv::Size(), 0.25, 0.25, cv::INTER_LANCZOS4 );
            m_reportDialog->addImage( resized );

        }

    }

    m_reportDialog->addBreak();
    m_reportDialog->addBreak();

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


/*    for ( auto &i : views ) {
        CvImage undistorted;

        cv::undistort( i, undistorted, cameraMatrix, distCoeffs );

        CvImage resizedView;
        cv::resize( undistorted, resizedView, cv::Size(), 0.5, 0.5, cv::INTER_LANCZOS4 );

        m_reportDialog->addImage( resizedView );
        m_reportDialog->addBreak();
    }*/

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
    if ( m_taskWidget->isTemplateExist() )
        m_iconsWidget->insertIcon( new StereoIcon(
                                        StereoCameraWidget::makeOverlappedPreview( m_taskWidget->leftDisplayedImage(), m_taskWidget->rightDisplayedImage() ),
                                        StereoCameraWidget::makeStraightPreview( m_taskWidget->leftDisplayedImage(), m_taskWidget->rightDisplayedImage() ),
                                        m_taskWidget->leftSourceImage(), m_taskWidget->rightSourceImage(), this ) );
}

void StereoCalibrationWidget::calculate()
{
    QDomElement docHead;

    m_processor.setCount( m_taskWidget->templateCount() );
    m_processor.setSize( m_taskWidget->templateSize() );

    m_reportDialog->showMaximized();
    m_reportDialog->activateWindow();

    m_reportDialog->clear();

    m_reportDialog->addText( tr( "Stereo camera calibration." ) + "\n" );
    m_reportDialog->addBreak();
    docHead.setAttribute("name", "Stereo camera calibration");

    std::vector< std::vector< cv::Point2f > > leftPoints2d;
    std::vector< std::vector< cv::Point2f > > rightPoints2d;
    std::vector< std::vector< cv::Point3f > > points3d;

    auto icons = m_iconsWidget->icons();

    cv::Size imageSize;

    // Calculating image size
    for ( auto &i : icons ) {
        auto currentSize = i->toStereoIcon()->leftSourceImage().size();

        if ( imageSize.empty() )
            imageSize = currentSize;
        else
            if ( imageSize != currentSize ) {
                m_reportDialog->addText( tr( "Error: image sizes must be the same." ) + "\n" );
                return;
            }

    }

    for ( auto &i : icons ) {
        auto currentSize = i->toStereoIcon()->rightSourceImage().size();

        if ( imageSize.empty() )
            imageSize = currentSize;
        else
            if ( imageSize != currentSize ) {
                m_reportDialog->addText( tr( "Error: image sizes must be the same." ) + "\n" );
                return;
            }

    }

    std::vector< CvImage > views;

    for ( auto i = icons.begin(); i != icons.end(); ) {
        auto leftImage = (*i)->toStereoIcon()->leftSourceImage();
        auto rightImage = (*i)->toStereoIcon()->rightSourceImage();

        if ( !leftImage.empty() && !rightImage.empty() ) {
            CvImage leftView;
            CvImage rightView;

            std::vector<cv::Point2f > leftImagePoints;
            std::vector<cv::Point2f > rightImagePoints;
            m_processor.processFrame( leftImage, &leftView, &leftImagePoints );
            m_processor.processFrame( rightImage, &rightView, &rightImagePoints );

            if ( !leftImagePoints.empty() && !rightImagePoints.empty() ) {

                std::vector< cv::Point3f > objectPoints;
                m_processor.calcChessboardCorners( &objectPoints );

                if ( !objectPoints.empty() ) {
                    views.push_back( StereoCameraWidget::makeStraightPreview( leftView, rightView ) );
                    leftPoints2d.push_back( leftImagePoints );
                    rightPoints2d.push_back( rightImagePoints );
                    points3d.push_back( objectPoints );
                }
                else {
                    i = icons.erase( i );
                    continue;

                }

            }
            else {
                i = icons.erase( i );
                continue;
            }

        }
        else {
            i = icons.erase( i );
            continue;
        }


        ++i;
    }

    if ( leftPoints2d.size() > 0 && rightPoints2d.size() > 0 && points3d.size() > 0 ) {

        cv::Mat leftCameraMatrix = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat rightCameraMatrix = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat leftDistCoeffs = cv::Mat::zeros(8, 1, CV_32F);
        cv::Mat rightDistCoeffs = cv::Mat::zeros(8, 1, CV_32F);
        cv::Mat R;
        cv::Mat T;
        cv::Mat E;
        cv::Mat F;

        double rms = cv::stereoCalibrate( points3d, leftPoints2d, rightPoints2d, leftCameraMatrix, leftDistCoeffs, rightCameraMatrix, rightDistCoeffs, imageSize,
                                          R, T, E, F, 0 );

        bool ok = cv::checkRange(leftCameraMatrix) && cv::checkRange(rightCameraMatrix) && cv::checkRange(leftDistCoeffs) && cv::checkRange(rightDistCoeffs);


        for (  int i = 0; i < views.size(); ++i ) {
            auto view = views[i];

            if ( !view.empty() ) {

                m_reportDialog->addText( tr( "Image" ) + " " + QString::number( i + 1 ) + "\n" );
                CvImage resizedView;
                cv::resize( view, resizedView, cv::Size(), 0.25, 0.25, cv::INTER_LANCZOS4 );
                m_reportDialog->addImage( resizedView );
                m_reportDialog->addDoubleBreak();

            }

        }

        if ( ok ) {

            m_reportDialog->addText( tr( "Left camera matrix:" ) + "\n" );
            m_reportDialog->addMatrix( leftCameraMatrix );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right camera matrix:" ) + "\n" );
            m_reportDialog->addMatrix( rightCameraMatrix );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Left distorsion coefficients:" ) + " " );
            m_reportDialog->addMatrix( leftDistCoeffs.t() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right distorsion coefficients:" ) + " " );
            m_reportDialog->addMatrix( rightDistCoeffs.t() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Rotation matrix:" ) + " " );
            m_reportDialog->addMatrix( R );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Translation vector:" ) + " " );
            m_reportDialog->addMatrix( T.t() );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Fundamental matrix:" ) + " " );
            m_reportDialog->addMatrix( F );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Essential matrix:" ) + " " );
            m_reportDialog->addMatrix( E );
            m_reportDialog->addDoubleBreak();

            cv::Mat leftRMap, leftDMap, rightRMap, rightDMap;

            cv::Mat R1, R2, P1, P2, Q;

            cv::Rect leftValidROI;
            cv::Rect rightValidROI;

            cv::stereoRectify( leftCameraMatrix, leftDistCoeffs, rightCameraMatrix, rightDistCoeffs, imageSize,
                               R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imageSize, &leftValidROI, &rightValidROI );

            m_reportDialog->addText( tr( "Left camera rotation matrix:" ) + " " );
            m_reportDialog->addMatrix( R1 );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Left camera projection matrix:" ) + " " );
            m_reportDialog->addMatrix( P1 );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right camera rotation matrix:" ) + " " );
            m_reportDialog->addMatrix( R2 );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right camera projection matrix:" ) + " " );
            m_reportDialog->addMatrix( P2 );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Left valid ROI:" ) + " " );
            m_reportDialog->addRect( leftValidROI );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Right valid ROI:" ) + " " );
            m_reportDialog->addRect( rightValidROI );
            m_reportDialog->addDoubleBreak();

            m_reportDialog->addText( tr( "Reprojection error:" ) +  " " + QString::number( rms ) +"\n" );
            m_reportDialog->addBreak();

            m_reportDialog->addText( tr( "Calibration succesful!" ) + "\n" );
            m_reportDialog->addDoubleBreak();

            cv::initUndistortRectifyMap( leftCameraMatrix, leftDistCoeffs, R1, P1, imageSize, CV_16SC2, leftRMap, leftDMap );
            cv::initUndistortRectifyMap( rightCameraMatrix, rightDistCoeffs, R2, P2, imageSize, CV_16SC2, rightRMap, rightDMap );

            for ( auto i = icons.begin(); i != icons.end(); ++i ) {
                auto leftImage = (*i)->toStereoIcon()->leftSourceImage();
                auto rightImage = (*i)->toStereoIcon()->rightSourceImage();

                if ( !leftImage.empty() && !rightImage.empty() ) {

                    cv::Mat leftRectifiedImage;
                    cv::Mat rightRectifiedImage;

                    cv::remap( leftImage, leftRectifiedImage, leftRMap, leftDMap, cv::INTER_LANCZOS4 );
                    cv::remap( rightImage, rightRectifiedImage, rightRMap, rightDMap, cv::INTER_LANCZOS4 );

                    cv::flip( leftRectifiedImage, leftRectifiedImage, -1 );
                    cv::flip( rightRectifiedImage, rightRectifiedImage, -1 );

                    auto preview = StereoCameraWidget::makeStraightPreview( leftRectifiedImage, rightRectifiedImage);
                    CvImage resizedView;
                    cv::resize( preview, resizedView, cv::Size(), 0.25, 0.25, cv::INTER_LANCZOS4 );

                    m_reportDialog->addImage( resizedView );

                }

            }

        }
        else
            m_reportDialog->addText( tr( "Calibration error!" ) + "\n" );

    }

}


