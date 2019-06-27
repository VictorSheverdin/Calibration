#pragma once

#include <QListWidget>
#include <memory>

#include "src/common/imagewidget.h"

class QLabel;
class QBoxLayout;

class IconBase : public QListWidgetItem
{

public:
    using SuperClass = QListWidget;

    IconBase( const CvImage image, const QString &text );

    const CvImage &previewImage() const;

protected:
    CvImage m_previewImage;

private:
    void initialize();

};

class IconsListWidget : public QListWidget
{
    Q_OBJECT

    friend class DisparityIcon;

public:
    using SuperClass = QListWidget;

    explicit IconsListWidget( QWidget *parent = nullptr );

    void addIcon( IconBase *icon );
    void insertIcon( IconBase *icon );

    QList< IconBase* > icons() const;

protected:
    static const QSize m_iconSize;

private:
    void initialize();

};
