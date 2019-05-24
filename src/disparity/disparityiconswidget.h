#pragma once

#include "src/common/iconswidget.h"

class DisparityIcon : public IconBase
{

public:
    using SuperClass = IconBase;

    DisparityIcon( const CvImage &preivewImage, const QString &leftFileName, const QString &rightFileName, const int number );

    void setLeftFileName( const QString &fileName );
    void setRightFileName( const QString &fileName );

    const QString &leftFileName() const;
    const QString &rightFileName() const;

    CvImage loadLeftImage() const;
    CvImage loadRightImage() const;

protected:
    QString m_leftFileName;
    QString m_rightFileName;

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
