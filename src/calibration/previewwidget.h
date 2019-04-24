#pragma once

#include <QThread>
#include <QSplitter>

#include "src/common/imagewidget.h"
#include "templateprocessor.h"

#include <opencv2/opencv.hpp>

class PreviewWidget : public ImageWidget
{
    Q_OBJECT

public:
    PreviewWidget( QWidget* parent = nullptr );

    const CvImage sourceImage() const;
    const CvImage previewImage() const;
    const std::vector<cv::Point2f> &previewPoints() const;

    bool isTemplateExist() const;

public slots:
    void setSourceImage( const CvImage image );
    void setPreviewImage( const CvImage image );
    void setPreviewPoints( const std::vector<cv::Point2f> &points );

protected:
    CvImage m_sourceImage;

    std::vector<cv::Point2f> m_previewPoints;

private:
    void initialize();

};
