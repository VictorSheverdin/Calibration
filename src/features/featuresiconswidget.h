#pragma once

#include "src/common/iconswidget.h"

class FeaturesIcon : public IconBase
{
public:
    using SuperClass = IconBase;

    FeaturesIcon(const CvImage &preivewImage, const QString &leftFileName, const QString &rightFileName, const QString &text );

    void setLeftFileName( const QString &fileName );
    void setRightFileName( const QString &fileName );

    const QString &leftFileName() const;
    const QString &rightFileName() const;

    CvImage loadLeftImage() const;
    CvImage loadRightImage() const;

    void setTime( const std::chrono::time_point< std::chrono::system_clock > &time );
    const std::chrono::time_point< std::chrono::system_clock > &time() const;

protected:
    QString m_fileName1;
    QString m_fileName2;

    std::chrono::time_point< std::chrono::system_clock > m_time;

private:
    void initialize();

};

class FeaturesIconsWidget : public IconsListWidget
{
    Q_OBJECT

public:
    using SuperClass = IconsListWidget;

    explicit FeaturesIconsWidget( QWidget *parent = nullptr );

    void addIcon( FeaturesIcon *icon );
    void insertIcon( FeaturesIcon *icon );

    QList< FeaturesIcon* > icons() const;

    FeaturesIcon *currentIcon() const;

signals:
    void iconActivated( FeaturesIcon *icon );

private:
    void initialize();

};
