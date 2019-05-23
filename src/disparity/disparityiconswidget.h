#pragma once

#include "src/common/iconswidget.h"

class DisparityIcon : public IconBase
{

public:
    using SuperClass = IconBase;

    DisparityIcon( const CvImage leftSourceImage, const CvImage rightSourceImage, const int number );

    void setLeftImage( const CvImage &image );
    void setRightImage( const CvImage &image );

    const CvImage &leftImage() const;
    const CvImage &rightImage() const;

protected:
    CvImage m_leftImage;
    CvImage m_rightImage;

private:
    void initialize();

};

class DisparityIconsWidget : public IconsListWidget
{
    Q_OBJECT

public:
    using SuperClass = IconsListWidget;

    explicit DisparityIconsWidget( QWidget *parent = nullptr );

    void addIcon( DisparityIcon *icon );
    void insertIcon( DisparityIcon *icon );

    QList< DisparityIcon* > icons() const;

    DisparityIcon *currentIcon() const;

signals:
    void iconActivated( DisparityIcon *icon );

private:
    void initialize();

};
