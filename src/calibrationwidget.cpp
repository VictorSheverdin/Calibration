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

void CalibrationWidgetBase::showIcon( IconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->previewImage() );

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

void MonocularCalibrationWidget::grabFrame()
{
    m_iconsWidget->insertIcon( new MonocularIcon( m_taskWidget->previewImage(), m_taskWidget->sourceImage(), this ) );
}

void MonocularCalibrationWidget::calculate()
{
    TemplateProcessor processor;
    processor.setCount(cv::Size(9,6));

    m_reportDialog->show();
    m_reportDialog->activateWindow();

    m_reportDialog->clear();

    m_reportDialog->addText( "Calibration report.\n" );

    std::vector< std::vector< cv::Point2f > > points2d;
    std::vector< std::vector< cv::Point3f > > points3d;

    auto icons = m_iconsWidget->icons();

    cv::Size imageSize;

    for (  int i = 0; i < icons.size(); ++i ) {
        auto icon = icons[i]->toMonocularIcon();

        auto image = icon->sourceImage();

        if ( !image.empty() ) {
            CvImage view;

            std::vector<cv::Point2f> imagePoints;
            processor.processFrame( image, &view, &imagePoints );

            imageSize = icon->sourceImage().size();

            if ( !imagePoints.empty() ) {

                std::vector< cv::Point3f > objectPoints;
                processor.calcChessboardCorners( &objectPoints );

                if ( !objectPoints.empty() ) {
                    points2d.push_back( imagePoints );
                    points3d.push_back( objectPoints );
                }
            }

            m_reportDialog->addText( "Image " + QString::number( i ) + "\n" );
            m_reportDialog->addImage( view );
            m_reportDialog->addBreak();

        }

    }

    if ( points2d.size() > 0 && points3d.size() > 0 ) {

        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
        std::vector< cv::Mat > rvecs;
        std::vector< cv::Mat > tvecs;

        double rms = cv::calibrateCamera( points3d, points2d, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 );

        bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

        m_reportDialog->addMatrix( cameraMatrix );

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

void StereoCalibrationWidget::grabFrame()
{
    m_iconsWidget->insertIcon( new StereoIcon(
                                        StereoCameraWidget::createPreview( m_taskWidget->leftDisplayedImage(), m_taskWidget->rightDisplayedImage() ),
                                        m_taskWidget->leftSourceImage(), m_taskWidget->rightSourceImage(), this ) );
}

void StereoCalibrationWidget::calculate()
{
    TemplateProcessor processor;

    m_reportDialog->show();
    m_reportDialog->activateWindow();

    m_reportDialog->clear();

    m_reportDialog->addText( "Calibration report.\n" );

    auto icons = m_iconsWidget->icons();

    for ( auto &i : icons ) {
        m_reportDialog->addIcon( *i );
    }


}


