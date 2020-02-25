#pragma once

#include <QThread>
#include <QSplitter>

#include "src/common/imagewidget.h"
#include "templatethread.h"

#include <opencv2/opencv.hpp>

class CameraPreviewWidget : public ImageWidget
{
    Q_OBJECT

public:
    CameraPreviewWidget( QWidget *parent = nullptr );

    const CvImage sourceImage() const;
    const CvImage previewImage() const;

    void setTemplateExist( const bool value );
    bool isTemplateExist() const;

public slots:
    void setSourceImage( const CvImage image );
    void setPreviewImage( const CvImage image );

protected:
    CvImage m_sourceImage;

    bool m_templateExist;

private:
    void initialize();

};
