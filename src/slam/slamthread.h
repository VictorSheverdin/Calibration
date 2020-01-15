#pragma once

#include <QThread>

#include <memory>

#include "slamgeometry.h"

#include "src/common/image.h"

namespace slam {
    class World;
}

class SlamThread : public QThread
{
    Q_OBJECT

public:
    explicit SlamThread( QObject *parent = nullptr );

signals:
    void pointsImageSignal( const CvImage &image );
    void tracksImageSignal( const CvImage &image );

    void geometrySignal( const SlamGeometry &geomtery );

    void updateSignal();

protected:
    std::string m_path;

    double m_scaleFactor;

    std::shared_ptr< slam::World > m_system;

    virtual void run() override;

private:
    void initialize();

};
