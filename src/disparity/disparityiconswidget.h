#pragma once

#include "src/common/iconswidget.h"

class DisparityIconBase : public IconBase
{
public:
    using SuperClass = IconBase;

    DisparityIconBase( const CvImage image, const QString &text );
};

class DisparityIcon : public DisparityIconBase
{
public:
    using SuperClass = DisparityIconBase;

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

class DisparityResultIcon : public DisparityIconBase
{

public:
    using SuperClass = DisparityIconBase;

    DisparityResultIcon( const CvImage &preivewImage, const QString &colorFileName, const QString &disparityFileName, const QString &text );

    void setColorFileName( const QString &fileName );
    void setDisparityFileName( const QString &fileName );

    const QString &colorFileName() const;
    const QString &disparityFileName() const;

    CvImage colorImage() const;
    CvImage disparityImage() const;

    void setTime( const std::chrono::time_point< std::chrono::system_clock > &time );
    const std::chrono::time_point< std::chrono::system_clock > &time() const;

protected:
    QString m_colorFileName;
    QString m_disparityFileName;

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

    void addIcon( DisparityIconBase *icon );
    void insertIcon( DisparityIconBase *icon );

    QList< DisparityIconBase* > icons() const;

    DisparityIconBase *currentIcon() const;

signals:
    void iconActivated( DisparityIconBase *icon );

private:
    void initialize();

};
