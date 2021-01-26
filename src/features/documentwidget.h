#pragma once

#include <QWidget>

#include "src/common/supportwidgets.h"

class QVBoxLayout;

class FeaturesWidget;

class FeaturesDocument : public DocumentBase
{
    Q_OBJECT

public:
    explicit FeaturesDocument( QWidget* parent = nullptr );

    FeaturesWidget *widget() const;

public slots:
    void importDialog();

private:
    void initialize();

};
