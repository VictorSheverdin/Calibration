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

MonocularCalibrationData CalibrationWidgetBase::calcMonocularCalibration( const QList< CalibrationIconBase *> &icons )
{
    MonocularCalibrationData ret;

    if ( !icons.empty() ) {

        cv::Size frameSize;

        std::vector< MonocularCalibrationResult > results;

        std::vector< std::vector< cv::Point2f > > points2d;
        std::vector< std::vector< cv::Point3f > > points3d;

        for ( auto &i : icons ) {

            auto currentIcon = i->toMonocularIcon();

            if ( currentIcon ) {

                auto currentPoints = currentIcon->imagePoints();
                auto objectPoints = currentIcon->worldPoints();

                if ( !currentPoints.empty() && !objectPoints.empty() ) {

                    points2d.push_back( currentPoints );
                    points3d.push_back( objectPoints );

                    MonocularCalibrationResult result;
                    result.setOk( true );
                    results.push_back( result );

                }

                if ( frameSize.empty() )
                    frameSize = currentIcon->frameSize();
                else if ( frameSize != currentIcon->frameSize() )
                    throw std::exception();

            }

        }

        ret = calcMonocularCalibration( points2d, points3d, frameSize );

        ret.setPreviewImage( icons.front()->previewImage() );

    }

    return ret;

}

MonocularCalibrationData CalibrationWidgetBase::calcMonocularCalibration( const std::vector< std::vector< cv::Point2f > > &points2d, const std::vector< std::vector< cv::Point3f > > &points3d, cv::Size &frameSize )
{
    MonocularCalibrationData ret;

    ret.setFrameSize( frameSize );

    std::vector< MonocularCalibrationResult > results;

    if ( points2d.size() < m_minimumCalibrationFrames || points2d.size() != points3d.size() )
        throw std::exception();

    cv::Mat cameraMatrix = cv::Mat::eye( 3, 3, CV_64F );
    cv::Mat distCoeffs = cv::Mat::zeros( 8, 1, CV_64F );
    std::vector< cv::Mat > rvecs;
    std::vector< cv::Mat > tvecs;

    double rms = cv::calibrateCamera( points3d, points2d, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 );

    bool ok = cv::checkRange( cameraMatrix ) && cv::checkRange( distCoeffs );

    ret.setCameraMatrix( cameraMatrix );
    ret.setDistortionCoefficients( distCoeffs );

    for ( size_t i = 0; i < rvecs.size(); ++i ) {
        MonocularCalibrationResult result;
        result.setRVec( rvecs[ i ] );
        result.setTVec( tvecs[ i ] );

        results.push_back( result );
    }

    ret.setResults( results );
    ret.setError( rms );
    ret.setOk( ok );

    return ret;

}

StereoCalibrationData CalibrationWidgetBase::calcStereoCalibration(const QList< CalibrationIconBase * > &icons )
{
    StereoCalibrationData ret;

    std::vector< std::vector< cv::Point3f > > points3d;
    std::vector< std::vector< cv::Point2f > > leftPoints;
    std::vector< std::vector< cv::Point2f > > rightPoints;

    cv::Size frameSize;

    for ( auto i = icons.begin(); i != icons.end(); ++i ) {

        auto currentIcon = (*i)->toStereoIcon();

        auto objectPoints = currentIcon->worldPoints();
        auto currentLeftPoints = currentIcon->leftImagePoints();
        auto currentRightPoints = currentIcon->rightImagePoints();

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

    ret = calcStereoCalibration( leftPoints, rightPoints, points3d, frameSize );

    ret.leftCameraResults().setPreviewImage( icons.front()->toStereoIcon()->leftPreview() );
    ret.rightCameraResults().setPreviewImage( icons.front()->toStereoIcon()->rightPreview() );

    return ret;

}

StereoCalibrationData CalibrationWidgetBase::calcStereoCalibration( const std::vector< std::vector< cv::Point2f > > &leftPoints, const std::vector< std::vector< cv::Point2f > > &rightPoints, const std::vector<std::vector<cv::Point3f> > &points3d, cv::Size &frameSize )
{
    StereoCalibrationData ret;

    if ( leftPoints.size() != rightPoints.size() || leftPoints.size() != points3d.size() || leftPoints.size() < m_minimumCalibrationFrames )
        throw std::exception();

    ret.setLeftCameraResults( calcMonocularCalibration( leftPoints, points3d, frameSize ) );
    ret.setRightCameraResults( calcMonocularCalibration( rightPoints, points3d, frameSize ) );

    ret.setCorrespondFrameCount( points3d.size() );

    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    cv::Mat rectifiedLeftCameraMatrix, rectifiedRightCameraMatrix;
    cv::Mat rectifiedLeftDistortionCoefficients, rectifiedRightDistortionCoefficients;

    ret.leftCameraResults().cameraMatrix().copyTo( rectifiedLeftCameraMatrix );
    ret.rightCameraResults().cameraMatrix().copyTo( rectifiedRightCameraMatrix );

    ret.leftCameraResults().distortionCoefficients().copyTo( rectifiedLeftDistortionCoefficients );
    ret.rightCameraResults().distortionCoefficients().copyTo( rectifiedRightDistortionCoefficients );

    double rms = cv::stereoCalibrate( points3d, leftPoints, rightPoints,
                                      rectifiedLeftCameraMatrix, rectifiedLeftDistortionCoefficients,
                                      rectifiedRightCameraMatrix, rectifiedRightDistortionCoefficients, ret.leftCameraResults().frameSize(),
                                      R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5,
                                      cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5 ) );

    ret.setRotationMatrix( R );
    ret.setTranslationVector( T );
    ret.setFundamentalMatrix( F );
    ret.setEssentialMatrix( E );
    ret.setError( rms );

    cv::Mat R1, R2, P1, P2, Q;

    cv::Rect leftROI;
    cv::Rect rightROI;

    cv::stereoRectify( rectifiedLeftCameraMatrix, rectifiedLeftDistortionCoefficients,
                       rectifiedRightCameraMatrix, rectifiedRightDistortionCoefficients, ret.leftCameraResults().frameSize(),
                       R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, cv::Size(), &leftROI, &rightROI );


    ret.setLeftCameraMatrix( rectifiedLeftCameraMatrix );
    ret.setRightCameraMatrix( rectifiedRightCameraMatrix );

    ret.setLeftDistortionCoefficients( rectifiedLeftDistortionCoefficients );
    ret.setRightDistortionCoefficients( rectifiedRightDistortionCoefficients );

    ret.setLeftRectifyMatrix( R1 );
    ret.setRightRectifyMatrix( R2 );
    ret.setLeftProjectionMatrix( P1 );
    ret.setRightProjectionMatrix( P2 );
    ret.setLeftROI( leftROI );
    ret.setRightROI( rightROI );

    ret.setOk( true );

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
    m_processorThread.templateProcessor().setAdaptiveThreshold( true );
    m_processorThread.templateProcessor().setFastCheck( false );
    m_processorThread.templateProcessor().setFilterQuads( true );
    m_processorThread.templateProcessor().setNormalizeImage( true );

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
    m_processorThread.templateProcessor().setAdaptiveThreshold( true );
    m_processorThread.templateProcessor().setFastCheck( false );
    m_processorThread.templateProcessor().setFilterQuads( true );
    m_processorThread.templateProcessor().setNormalizeImage( true );
}

void StereoCalibrationWidgetBase::showIcon( CalibrationIconBase *icon )
{
    m_iconViewDialog->show();
    m_iconViewDialog->activateWindow();
    m_iconViewDialog->setImage( icon->toStereoIcon()->stackedPreview() );
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
                            "Image files (*.png *.xpm *.jpg *.tiff *.exr)" );

    for ( auto &i : files )
        loadIcon( i );

}

void MonocularImageCalibrationWidget::exportDialog()
{
}

void MonocularImageCalibrationWidget::calculate()
{
    auto calibrationResult = calcMonocularCalibration( m_iconsList->icons() );

    auto doc = application()->mainWindow()->addMonocularReportDocument();

    doc->report( calibrationResult );

}

MonocularIcon *MonocularImageCalibrationWidget::createIcon( const CvImage &image )
{
    if ( m_parametersWidget->templateType() == TypeComboBox::CHECKERBOARD )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
    else if ( m_parametersWidget->templateType() == TypeComboBox::CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CIRCLES );
    else if ( m_parametersWidget->templateType() == TypeComboBox::ASYM_CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );

    m_processorThread.templateProcessor().setCount( m_parametersWidget->templateCount() );
    m_processorThread.templateProcessor().setSize( m_parametersWidget->templateSize() );

    m_processorThread.markerProcessor().setSize( m_parametersWidget->templateSize() );

    MonocularProcessorResult result;

    if ( m_parametersWidget->templateType() == TypeComboBox::CHECKERBOARD || m_parametersWidget->templateType() == TypeComboBox::CIRCLES || m_parametersWidget->templateType() == TypeComboBox::ASYM_CIRCLES  )
        result = m_processorThread.calculate( image, MonocularProcessorThread::TEMPLATE );
    else if ( m_parametersWidget->templateType() == TypeComboBox::ARUCO_MARKERS )
        result = m_processorThread.calculate( image, MonocularProcessorThread::MARKER );

    if ( result.exist && result.imagePoints.size() >= m_minimumCalibrationPoints ) {
        return new MonocularIcon( result.preview, result.sourceFrame.size(),
                                                    result.imagePoints, result.worldPoints, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) );

    }


    return nullptr;
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
    auto calibrationResult = calcStereoCalibration( m_iconsList->icons() );

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
   StampedStereoImage frame( leftImage, rightImage );

    if ( m_parametersWidget->templateType() == TypeComboBox::CHECKERBOARD )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
    else if ( m_parametersWidget->templateType() == TypeComboBox::CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CIRCLES );
    else if ( m_parametersWidget->templateType() == TypeComboBox::ASYM_CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );

    m_processorThread.templateProcessor().setCount( m_parametersWidget->templateCount() );
    m_processorThread.templateProcessor().setSize( m_parametersWidget->templateSize() );

    m_processorThread.markerProcessor().setSize( m_parametersWidget->templateSize() );

    StereoProcessorResult result;

    if ( m_parametersWidget->templateType() == TypeComboBox::CHECKERBOARD || m_parametersWidget->templateType() == TypeComboBox::CIRCLES || m_parametersWidget->templateType() == TypeComboBox::ASYM_CIRCLES  )
        result = m_processorThread.calculate( frame, StereoProcessorThread::TEMPLATE );
    else if ( m_parametersWidget->templateType() == TypeComboBox::ARUCO_MARKERS )
        result = m_processorThread.calculate( frame, StereoProcessorThread::MARKER );

    if ( result.leftExist && result.rightExist && result.leftImagePoints.size() == result.rightImagePoints.size()
                && result.leftImagePoints.size() >= m_minimumCalibrationPoints ) {
        return new StereoIcon( result.leftPreview, result.rightPreview,
                               result.sourceFrame.leftImage().size(), result.leftImagePoints, result.rightImagePoints, result.worldPoints,
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

    m_processorThread.start();

    connect( &m_processorThread, &MonocularProcessorThread::updateSignal, this, &MonocularCameraCalibrationWidget::makeIcon );

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
                            "Image files (*.png *.xpm *.jpg *.tiff *.exr)" );

    for ( auto &i : files )
        loadIcon( i );

}

void MonocularCameraCalibrationWidget::exportDialog()
{

}

void MonocularCameraCalibrationWidget::calculate()
{
    auto calibrationResult = calcMonocularCalibration( m_iconsList->icons() );

    auto doc = application()->mainWindow()->addMonocularReportDocument();

    doc->report( calibrationResult );

}

void MonocularCameraCalibrationWidget::grabFrame()
{
    auto taskWidget = this->taskWidget();

    auto image = taskWidget->sourceImage();

    if ( taskWidget->isTemplateExist() ) {

        if ( m_taskWidget->templateType() == TypeComboBox::CHECKERBOARD )
            m_processorThread.templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
        else if ( m_taskWidget->templateType() == TypeComboBox::CIRCLES )
            m_processorThread.templateProcessor().setType( TemplateProcessor::CIRCLES );
        else if ( m_taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES )
            m_processorThread.templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );

        m_processorThread.templateProcessor().setCount( m_taskWidget->templateCount() );
        m_processorThread.templateProcessor().setSize( m_taskWidget->templateSize() );

        m_processorThread.markerProcessor().setSize( m_taskWidget->templateSize() );

        if ( taskWidget->templateType() == TypeComboBox::CHECKERBOARD || taskWidget->templateType() == TypeComboBox::CIRCLES || taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES  )
            m_processorThread.processFrame( image, MonocularProcessorThread::TEMPLATE );
        else if ( taskWidget->templateType() == TypeComboBox::ARUCO_MARKERS )
            m_processorThread.processFrame( image, MonocularProcessorThread::MARKER );

    }

}

MonocularIcon *MonocularCameraCalibrationWidget::createIcon( const CvImage &image )
{
    auto taskWidget = this->taskWidget();

    if ( m_taskWidget->templateType() == TypeComboBox::CHECKERBOARD )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
    else if ( m_taskWidget->templateType() == TypeComboBox::CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CIRCLES );
    else if ( m_taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );

    m_processorThread.templateProcessor().setCount( m_taskWidget->templateCount() );
    m_processorThread.templateProcessor().setSize( m_taskWidget->templateSize() );

    m_processorThread.markerProcessor().setSize( m_taskWidget->templateSize() );

    MonocularProcessorResult result;

    if ( taskWidget->templateType() == TypeComboBox::CHECKERBOARD || taskWidget->templateType() == TypeComboBox::CIRCLES || taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES  )
        result = m_processorThread.calculate( image, MonocularProcessorThread::TEMPLATE );
    else if ( taskWidget->templateType() == TypeComboBox::ARUCO_MARKERS )
        result = m_processorThread.calculate( image, MonocularProcessorThread::MARKER );

    if ( result.exist && result.imagePoints.size() >= m_minimumCalibrationPoints ) {
        return new MonocularIcon( result.preview, result.sourceFrame.size(),
                                                    result.imagePoints, result.worldPoints, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) );

    }


    return nullptr;
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

void MonocularCameraCalibrationWidget::makeIcon()
{
    auto result = m_processorThread.result();

    if ( result.exist && result.imagePoints.size() >= m_minimumCalibrationPoints ) {
        m_iconsList->insertIcon( new MonocularIcon( result.preview, result.sourceFrame.size(),
                                                    result.imagePoints, result.worldPoints, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) ) );

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

    m_processorThread.start();

    connect( &m_processorThread, &StereoProcessorThread::updateSignal, this, &StereoCameraCalibrationWidget::makeIcon );

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

    auto frame = taskWidget->sourceFrame();

    if ( taskWidget->isTemplateExist() ) {

        if ( m_taskWidget->templateType() == TypeComboBox::CHECKERBOARD )
            m_processorThread.templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
        else if ( m_taskWidget->templateType() == TypeComboBox::CIRCLES )
            m_processorThread.templateProcessor().setType( TemplateProcessor::CIRCLES );
        else if ( m_taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES )
            m_processorThread.templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );

        m_processorThread.templateProcessor().setCount( m_taskWidget->templateCount() );
        m_processorThread.templateProcessor().setSize( m_taskWidget->templateSize() );

        m_processorThread.markerProcessor().setSize( m_taskWidget->templateSize() );

        if ( taskWidget->templateType() == TypeComboBox::CHECKERBOARD || taskWidget->templateType() == TypeComboBox::CIRCLES || taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES  )
            m_processorThread.processFrame( frame, StereoProcessorThread::TEMPLATE );
        else if ( taskWidget->templateType() == TypeComboBox::ARUCO_MARKERS )
            m_processorThread.processFrame( frame, StereoProcessorThread::MARKER );

    }

}

void StereoCameraCalibrationWidget::calculate()
{
    auto calibrationResult = calcStereoCalibration( m_iconsList->icons() );

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

void StereoCameraCalibrationWidget::makeIcon()
{
    auto result = m_processorThread.result();

    if ( result.leftExist && result.rightExist && result.leftImagePoints.size() == result.rightImagePoints.size()
         && result.leftImagePoints.size() >= m_minimumCalibrationPoints ) {
        m_iconsList->insertIcon( new StereoIcon( result.leftPreview, result.rightPreview,
                               result.sourceFrame.leftImage().size(), result.leftImagePoints, result.rightImagePoints, result.worldPoints,
                               QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) ) );

    }

}

StereoIcon *StereoCameraCalibrationWidget::createIcon( const CvImage &leftImage, const CvImage &rightImage )
{    
    auto taskWidget = this->taskWidget();

   StampedStereoImage frame( leftImage, rightImage );

    if ( m_taskWidget->templateType() == TypeComboBox::CHECKERBOARD )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CHECKERBOARD );
    else if ( m_taskWidget->templateType() == TypeComboBox::CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::CIRCLES );
    else if ( m_taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES )
        m_processorThread.templateProcessor().setType( TemplateProcessor::ASYM_CIRCLES );

    m_processorThread.templateProcessor().setCount( m_taskWidget->templateCount() );
    m_processorThread.templateProcessor().setSize( m_taskWidget->templateSize() );

    m_processorThread.markerProcessor().setSize( m_taskWidget->templateSize() );

    StereoProcessorResult result;

    if ( taskWidget->templateType() == TypeComboBox::CHECKERBOARD || taskWidget->templateType() == TypeComboBox::CIRCLES || taskWidget->templateType() == TypeComboBox::ASYM_CIRCLES  )
        result = m_processorThread.calculate( frame, StereoProcessorThread::TEMPLATE );
    else if ( taskWidget->templateType() == TypeComboBox::ARUCO_MARKERS )
        result = m_processorThread.calculate( frame, StereoProcessorThread::MARKER );

    if ( result.leftExist && result.rightExist && result.leftImagePoints.size() == result.rightImagePoints.size()
         && result.leftImagePoints.size() >= m_minimumCalibrationPoints ) {
        return new StereoIcon( result.leftPreview, result.rightPreview,
                               result.sourceFrame.leftImage().size(), result.leftImagePoints, result.rightImagePoints, result.worldPoints,
                               QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) );

    }

    return nullptr;

}
