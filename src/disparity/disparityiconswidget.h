#pragma once

#include "src/common/iconswidget.h"

class DisparityIcon : public IconBase
{

public:
    using SuperClass = IconBase;

    DisparityIcon(const CvImage &preivewImage, const QString &leftFileName, const QString &rightFileName, const QString &text );

    void setLeftFileName( const QString &fileName );
    void setRightFileName( const QString &fileName );

    const QString &leftFileName() const;
    const QString &rightFileName() const;

    CvImage loadLeftImage() const;
    CvImage loadRightImage() const;

    StampedStereoImage stereoFrame() const;

    void setTime( const std::chrono::time_point< std::chrono::system_clock > &time );
    const std::chrono::time_point< std::chrono::system_clock > &time() const;

protected:
    QString m_leftFileName;
    QString m_rightFileName;

    std::chrono::time_point< std::chrono::system_clock > m_time;

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
