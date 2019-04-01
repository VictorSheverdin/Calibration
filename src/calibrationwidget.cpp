#include "precompiled.h"

#include "calibrationwidget.h"

#include "taskwidget.h"
#include "iconswidget.h"
#include "camerawidget.h"
#include "reportwidget.h"

#include "application.h"
#include "mainwindow.h"

CalibrationWidgetBase::CalibrationWidgetBase( QWidget *parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void CalibrationWidgetBase::initialize()
{
    m_iconViewDialog = new ImageDialog( application()->mainWindow() );
    m_iconViewDialog->resize( 800, 600 );

    m_reportDialog = new ReportDialog( application()->mainWindow() );
    m_reportDialog->resize( 800, 600 );

}

// MonocularCalibrationWidget
MonocularCalibrationWidget::MonocularCalibrationWidget( const int cameraIndex, QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize( cameraIndex );
}

void MonocularCalibrationWidget::initialize( const int cameraIndex )
{
    m_taskWidget = new MonocularTaskWidget( cameraIndex, this );
    m_iconsWidget = new IconsWidget( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsWidget );

    connect( m_iconsWidget, SIGNAL( iconActivated( IconBase* ) ), this, SLOT( showIcon( IconBase* ) ) );

}

void MonocularCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->previewImage() );

}

void MonocularCalibrationWidget::grabFrame()
{
    m_iconsWidget->insertIcon( new MonocularIcon( m_taskWidget->previewImage(), m_taskWidget->sourceImage(), this ) );
}

void MonocularCalibrationWidget::calculate()
{
    TemplateProcessor processor;

    processor.setAdaptiveThreshold( true );
    processor.setFastCheck( false );
    processor.setFilterQuads( true );
    processor.setNormalizeImage( true );

    processor.setCount( cv::Size(9,6) );

    m_reportDialog->show();
    m_reportDialog->activateWindow();

    m_reportDialog->clear();

    m_reportDialog->addText( tr( "Camera calibration." ) + "\n" );
    m_reportDialog->addBreak();

    std::vector< std::vector< cv::Point2f > > points2d;
    std::vector< std::vector< cv::Point3f > > points3d;

    auto icons = m_iconsWidget->icons();

    cv::Size imageSize;

    // Calculating image size
    for ( auto &i : icons ) {
        auto currentSize = i->toMonocularIcon()->sourceImage().size();

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
        auto image = (*i)->toMonocularIcon()->sourceImage();

        if ( !image.empty() ) {
            CvImage view;

            std::vector<cv::Point2f> imagePoints;
            processor.processFrame( image, &view, &imagePoints );

            if ( !imagePoints.empty() ) {

                std::vector< cv::Point3f > objectPoints;
                processor.calcChessboardCorners( &objectPoints );

                if ( !objectPoints.empty() ) {
                    views.push_back( view );
                    points2d.push_back( imagePoints );
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

    if ( points2d.size() > 0 && points3d.size() > 0 ) {

        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
        std::vector< cv::Mat > rvecs;
        std::vector< cv::Mat > tvecs;

        double rms = cv::calibrateCamera( points3d, points2d, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs );

        bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);


        for (  int i = 0; i < views.size(); ++i ) {
            auto view = views[i];

            if ( !view.empty() ) {

                m_reportDialog->addText( tr( "Image" ) + " " + QString::number( i + 1 ) + "\n" );
                m_reportDialog->addImage( view );
                m_reportDialog->addBreak();
                m_reportDialog->addText( tr( "Rotation vector:" ) + " " );
                m_reportDialog->addMatrix( rvecs[ i ].t() );
                m_reportDialog->addText( tr( "Translation vector:" ) + " " );
                m_reportDialog->addMatrix( tvecs[ i ].t() );
                m_reportDialog->addBreak();
                m_reportDialog->addBreak();

            }

        }


        m_reportDialog->addText( tr( "Camera matrix:" ) + "\n" );
        m_reportDialog->addMatrix( cameraMatrix );
        m_reportDialog->addBreak();

        m_reportDialog->addBreak();
        m_reportDialog->addText( tr( "Distorsion coefficients:" ) + " " );
        m_reportDialog->addMatrix( distCoeffs.t() );
        m_reportDialog->addBreak();

        m_reportDialog->addBreak();
        m_reportDialog->addText( tr( "Reprojection error:" ) +  " " + QString::number( rms ) +"\n" );
        m_reportDialog->addBreak();

        if ( ok )
            m_reportDialog->addText( tr( "Calibration succesful!" ) + "\n" );
        else
            m_reportDialog->addText( tr( "Calibration error!" ) + "\n" );

    }

}

// StereoCalibrationWidget
StereoCalibrationWidget::StereoCalibrationWidget( const int leftCameraIndex, const int rightCameraIndex, QWidget *parent )
    : CalibrationWidgetBase( parent )
{
    initialize( leftCameraIndex, rightCameraIndex );
}

void StereoCalibrationWidget::initialize( const int leftCameraIndex, const int rightCameraIndex )
{
    m_taskWidget = new StereoTaskWidget( leftCameraIndex, rightCameraIndex, this );
    m_iconsWidget = new IconsWidget( this );

    addWidget( m_taskWidget );
    addWidget( m_iconsWidget );

    connect( m_iconsWidget, SIGNAL( iconActivated( IconBase* ) ), this, SLOT( showIcon( IconBase* ) ) );
}

void StereoCalibrationWidget::showIcon( IconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->toStereoIcon()->straightPreview() );

}

void StereoCalibrationWidget::grabFrame()
{
    m_iconsWidget->insertIcon( new StereoIcon(
                                        StereoCameraWidget::makeOverlappedPreview( m_taskWidget->leftDisplayedImage(), m_taskWidget->rightDisplayedImage() ),
                                        StereoCameraWidget::makeStraightPreview( m_taskWidget->leftDisplayedImage(), m_taskWidget->rightDisplayedImage() ),
                                        m_taskWidget->leftSourceImage(), m_taskWidget->rightSourceImage(), this ) );
}

void StereoCalibrationWidget::calculate()
{
    TemplateProcessor processor;

    processor.setAdaptiveThreshold( true );
    processor.setFastCheck( false );
    processor.setFilterQuads( true );
    processor.setNormalizeImage( true );

    processor.setCount( cv::Size(9,6) );

    m_reportDialog->show();
    m_reportDialog->activateWindow();

    m_reportDialog->clear();

    m_reportDialog->addText( tr( "Stereo camera calibration." ) + "\n" );
    m_reportDialog->addBreak();

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

            std::vector<cv::Point2f> leftImagePoints;
            std::vector<cv::Point2f> rightImagePoints;
            processor.processFrame( leftImage, &leftView, &leftImagePoints );
            processor.processFrame( rightImage, &rightView, &rightImagePoints );

            if ( !leftImagePoints.empty() && !rightImagePoints.empty() ) {

                std::vector< cv::Point3f > objectPoints;
                processor.calcChessboardCorners( &objectPoints );

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

        cv::Mat leftCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat rightCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat leftDistCoeffs = cv::Mat::zeros(8, 1, CV_64F);
        cv::Mat rightDistCoeffs = cv::Mat::zeros(8, 1, CV_64F);
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
                m_reportDialog->addImage( view );
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

                    m_reportDialog->addImage( StereoCameraWidget::makeStraightPreview( leftRectifiedImage, rightRectifiedImage ) );

                }

            }

        }
        else
            m_reportDialog->addText( tr( "Calibration error!" ) + "\n" );

    }

}


