#pragma once

#include <QWidget>
#include <QSplitter>

#include "src/common/defs.h"
#include "processorthread.h"

class ImageWidget;
class ImageViewer;
class FeaturesIcon;
class FeaturesIconsWidget;
class FeaturesControlWidget;

class FeaturesViewWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit FeaturesViewWidget( QWidget* parent = nullptr );

signals:
    void valueChanged();

public slots:
    void processFrame( const CvImage &img1, const CvImage &img2 );

protected slots:
    void updateFrame();

protected:
    QPointer< ImageViewer > m_imageWidget;
    QPointer< FeaturesControlWidget > m_controlWidget;

    std::shared_ptr< CPUFlowProcessor > m_flowProcessor;

    std::shared_ptr< GFTTProcessor > m_gfftProcessor;
    std::shared_ptr< FastProcessor > m_fastProcessor;

    std::shared_ptr< DaisyProcessor > m_daisyProcessor;
    std::shared_ptr< FreakProcessor > m_freakProcessor;
    std::shared_ptr< SiftProcessor > m_siftProcessor;
    std::shared_ptr< SurfProcessor > m_surfProcessor;
    std::shared_ptr< OrbProcessor > m_orbProcessor;
    std::shared_ptr< KazeProcessor > m_kazeProcessor;
    std::shared_ptr< AKazeProcessor > m_akazeProcessor;
    std::shared_ptr< SuperGlueProcessor > m_superglueProcessor;

    std::shared_ptr< DescriptorMatcher > m_matcher;

    ProcessorThread m_processorThread;

private:
    void initialize();
};

class FeaturesWidget : public QSplitter
{
    Q_OBJECT

public:
    explicit FeaturesWidget( QWidget* parent = nullptr );

public slots:
    void importDialog();

    void addIcon( const QString &leftFileName, const QString &rightFileName );
    void clearIcons();

protected slots:
    void updateFrame();
    void updateFrame( FeaturesIcon *icon );

protected:
    QPointer< FeaturesViewWidget > m_viewWidget;
    QPointer< FeaturesIconsWidget > m_iconsWidget;

    int m_iconCount;

    void dropIconCount();

private:
    void initialize();

};


