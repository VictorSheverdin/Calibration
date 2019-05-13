#pragma once

#include <QWidget>

#include "supportwidgets.h"

class CameraIPWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraIPWidget( QWidget* parent = nullptr );

private:
    void initialize();

};
