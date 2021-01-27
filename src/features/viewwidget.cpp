#include "src/common/precompiled.h"

#include "viewwidget.h"

#include "featuresiconswidget.h"

#include "controlwidget.h"

#include "src/common/imagewidget.h"
#include "src/common/fileslistwidget.h"
#include "src/common/functions.h"

#include "application.h"
#include "mainwindow.h"

// FeaturesViewWidget
FeaturesViewWidget::FeaturesViewWidget( QWidget* parent )
    : QSplitter( parent )
{
    initialize();
}

void FeaturesViewWidget::initialize()
{
    m_gfftProcessor = std::shared_ptr< GFTTProcessor >( new GFTTProcessor );
    m_gfftProcessor->setMaxFeatures( 10000 );
    m_gfftProcessor->setMinDistance( 10 );
    m_gfftProcessor->setBlockSize( 5 );
    m_fastProcessor = std::shared_ptr< FastProcessor >( new FastProcessor );

    m_flowProcessor = std::shared_ptr< CPUFlowProcessor >( new CPUFlowProcessor );
    m_daisyProcessor = std::shared_ptr< DaisyProcessor >( new DaisyProcessor );
    m_freakProcessor = std::shared_ptr< FreakProcessor >( new FreakProcessor );
    m_siftProcessor = std::shared_ptr< SiftProcessor >( new SiftProcessor );
    m_surfProcessor = std::shared_ptr< SurfProcessor >( new SurfProcessor );
    m_orbProcessor = std::shared_ptr< OrbProcessor >( new OrbProcessor );
    m_kazeProcessor = std::shared_ptr< KazeProcessor >( new KazeProcessor );
    m_akazeProcessor = std::shared_ptr< AKazeProcessor >( new AKazeProcessor );
    m_superglueProcessor = std::shared_ptr< SuperGlueProcessor >( new SuperGlueProcessor( "superpoint_fp32.eng", "superglue_fp32.eng" ) );

    m_matcher = std::shared_ptr< BFMatcher >( new BFMatcher );

    m_processorThread.setDetector( m_siftProcessor );
    m_processorThread.setDescriptor( m_siftProcessor );
    m_processorThread.setMatcher( m_matcher );

    m_imageWidget = new ImageViewer( this );
    m_imageWidget->setScale( 0.3 );

    m_controlWidget = new FeaturesControlWidget( this );

    addWidget( m_imageWidget );
    addWidget( m_controlWidget );

    connect( m_controlWidget, &FeaturesControlWidget::valueChanged, this, &FeaturesViewWidget::valueChanged );

    connect( &m_processorThread, &ProcessorThread::frameProcessed, this, &FeaturesViewWidget::updateFrame );

}

void FeaturesViewWidget::processFrame( const CvImage &img1, const CvImage &img2 )
{
    if ( m_controlWidget->isShiTomasiDetector() )
        m_processorThread.setDetector( m_gfftProcessor );
    else if ( m_controlWidget->isFastDetector() )
        m_processorThread.setDetector( m_fastProcessor );
    else if ( m_controlWidget->isSiftDetector() )
        m_processorThread.setDetector( m_siftProcessor );
    else if ( m_controlWidget->isSurfDetector() )
        m_processorThread.setDetector( m_surfProcessor );
    else if ( m_controlWidget->isOrbDetector() )
        m_processorThread.setDetector( m_orbProcessor );
    else if ( m_controlWidget->isKazeDetector() )
        m_processorThread.setDetector( m_kazeProcessor );
    else if ( m_controlWidget->isAkazeDetector() )
        m_processorThread.setDetector( m_akazeProcessor );
    else if ( m_controlWidget->isSuperGlueDetector() )
        m_processorThread.setDetector( m_superglueProcessor );

    if ( m_controlWidget->isLKFlow() )
        m_processorThread.setDescriptor( m_flowProcessor );
    else if ( m_controlWidget->isDaisyDescriptor() )
        m_processorThread.setDescriptor( m_daisyProcessor );
    else if ( m_controlWidget->isFreakDescriptor() )
        m_processorThread.setDescriptor( m_freakProcessor );
    else if ( m_controlWidget->isSiftDescriptor() )
        m_processorThread.setDescriptor( m_siftProcessor );
    else if ( m_controlWidget->isSurfDescriptor() )
        m_processorThread.setDescriptor( m_surfProcessor );
    else if ( m_controlWidget->isKazeDescriptor() )
        m_processorThread.setDescriptor( m_kazeProcessor );
    else if ( m_controlWidget->isAkazeDescriptor() )
        m_processorThread.setDescriptor( m_akazeProcessor );
    else if ( m_controlWidget->isSuperGlueDescriptor() )
        m_processorThread.setDescriptor( m_superglueProcessor );

    auto scaleFactor = m_controlWidget->scaleFactor();

    CvImage scaledImage1, scaledImage2;

    cv::resize( img1, scaledImage1, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
    cv::resize( img2, scaledImage2, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );

    m_processorThread.process( scaledImage1, scaledImage2 );
}

void FeaturesViewWidget::updateFrame()
{
    auto result = m_processorThread.result();
    m_imageWidget->setImage( result.result );
    application()->mainWindow()->statusBar()->showMessage( tr("Elapsed time: ") + QString::number( result.procTime ) + tr( ", count: " ) + QString::number( result.count ) );
}

// FeaturesWidget
FeaturesWidget::FeaturesWidget( QWidget* parent )
    : QSplitter( Qt::Vertical, parent )
{
    initialize();
}

void FeaturesWidget::initialize()
{
    m_viewWidget = new FeaturesViewWidget( this );
    m_iconsWidget = new FeaturesIconsWidget( this );

    addWidget( m_viewWidget );
    addWidget( m_iconsWidget );

    dropIconCount();

    connect( m_viewWidget, SIGNAL( valueChanged() ), this, SLOT( updateFrame() ) );
    connect( m_iconsWidget, SIGNAL( iconActivated( FeaturesIcon* ) ), this, SLOT( updateFrame() ) );

}

void FeaturesWidget::importDialog()
{
    StereoFilesListDialog dlg( this );

    if ( dlg.exec() == StereoFilesListDialog::Accepted ) {
        auto leftFileNames = dlg.leftFileNames();
        auto rightFileNames = dlg.rightFileNames();

        for ( auto i = 0; i < leftFileNames.size(); ++i )
            addIcon( leftFileNames[i], rightFileNames[i] );

    }

    updateFrame();

}

void FeaturesWidget::addIcon( const QString &leftFileName, const QString &rightFileName )
{
    CvImage leftImg = cv::imread( leftFileName.toStdString() );
    CvImage rightImg = cv::imread( rightFileName.toStdString() );

    if ( !leftImg.empty() && !rightImg.empty() )
        m_iconsWidget->addIcon( new FeaturesIcon( makeOverlappedPreview( leftImg, rightImg ) , leftFileName, rightFileName, QObject::tr("Frame") + " " + QString::number( m_iconCount++ ) ) );

}

void FeaturesWidget::clearIcons()
{
    m_iconsWidget->clear();

    dropIconCount();
}

void FeaturesWidget::updateFrame()
{
    auto icon = m_iconsWidget->currentIcon();

    auto featuresIcon = dynamic_cast< FeaturesIcon * >( icon );

    if ( featuresIcon )
        updateFrame( featuresIcon );

}

void FeaturesWidget::updateFrame( FeaturesIcon* icon )
{
    if ( icon )
        m_viewWidget->processFrame( icon->loadLeftImage(), icon->loadRightImage() );
}

void FeaturesWidget::dropIconCount()
{
    m_iconCount = 0;
}
