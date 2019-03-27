#pragma once

#include <QTextEdit>

#include "image.h"

class IconBase;

class ReportWidget : public QTextEdit
{
    Q_OBJECT

public:
    explicit ReportWidget( QWidget* parent = nullptr );

public slots:
    void addText( const QString &text );
    void addBreak();
    void addNumber( const int num );
    void addNumber( const double num );
    void addIcon( const IconBase& icon );
    void addImage( const CvImage& image );
    void addMatrix( const cv::Mat& mat );

protected:

private:
    void initialize();

};

class ReportDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ReportDialog( QWidget *parent = nullptr );

public slots:
    void clear();

    void addText( const QString &text );
    void addBreak();
    void addNumber( const int num );
    void addNumber( const double num );
    void addIcon( const IconBase& icon );
    void addImage( const CvImage& image );
    void addMatrix( const cv::Mat& mat );

protected:
    QPointer< ReportWidget > m_reportWidget;

private:
    void initialize();

};
