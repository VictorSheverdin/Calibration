#pragma once

#include <QObject>

#include "src/common/vimbacamera.h"

class Writer : public QObject
{
    Q_OBJECT

public:
    explicit Writer( const std::string &leftIp, const std::string &rightIp, QObject *parent= nullptr );

protected slots:
    void writeFrame();

protected:
    StereoCamera m_camera;

    int m_frameNumber;

private:
    void initialize();

};
