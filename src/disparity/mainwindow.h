#pragma once

#include <QMainWindow>

#include <QPointer>

#include "src/common/defs.h"

class PreviewWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( const std::string &leftCameraIp, const std::string &rightCameraIp, QWidget *parent = nullptr );

public slots:
    void loadCalibrationFile( const std::string &fileName );

protected:
    QPointer< PreviewWidget > m_widget;

private:
    void initialize(const std::string &leftCameraIp, const std::string &rightCameraIp );
    void initialize();

};
