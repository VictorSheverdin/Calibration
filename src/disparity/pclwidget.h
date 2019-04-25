#pragma once

#include <QWidget>
#include <QVTKWidget.h>

#include <vtkRenderWindow.h>

#include <pcl/visualization/pcl_visualizer.h>

class PCLViewer : public QVTKWidget
{
    Q_OBJECT

public:
    PCLViewer( QWidget* parent = nullptr )
        : QVTKWidget( parent )
    {
        pclviewer = new pcl::visualization::PCLVisualizer( "PCLVisualizer", false );
        SetRenderWindow(pclviewer->getRenderWindow());
        pclviewer->setupInteractor(GetInteractor(), GetRenderWindow());
        pclviewer->initCameraParameters ();

        this->initUi();
    }

    virtual ~PCLViewer()
    {
        this->hide();
        delete pclviewer;
    }


    virtual void initUi()
    {
        this->resize( 640, 480 );
        pclviewer->setBackgroundColor( 0, 0, 0 );
    }

public:
    pcl::visualization::PCLVisualizer* pclviewer;

};
