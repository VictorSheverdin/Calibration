#pragma once

#include <QListWidget>

#include "src/common/imagewidget.h"

class QLabel;
class QBoxLayout;

class IconBase : public QListWidgetItem
{

public:
    using SuperClass = QListWidget;

    IconBase( const CvImage image, const int number );

    const CvImage &previewImage() const;

protected:
    CvImage m_previewImage;

private:
    void initialize();

};

class IconsWidget : public QListWidget
{
    Q_OBJECT

public:
    using SuperClass = QListWidget;

    explicit IconsWidget( QWidget *parent = nullptr );

    void addIcon( IconBase *icon );
    void insertIcon( IconBase *icon );

    QList< IconBase* > icons() const;

protected:
    static const QSize m_iconSize;

private:
    void initialize();

};
