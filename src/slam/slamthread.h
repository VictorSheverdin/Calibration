#pragma once

#include <QThread>

#include <memory>

#include "slamgeometry.h"

#include "src/common/image.h"

namespace slam {
    class World;
    class Map;
}

class SlamThread : public QThread
{
    Q_OBJECT

public:
    using MapPtr = std::shared_ptr< slam::Map >;

    explicit SlamThread( QObject *parent = nullptr );

    const std::list < MapPtr > &maps() const;

    CvImage pointsImage() const;
    CvImage tracksImage() const;
    CvImage stereoImage() const;

signals:
    void updateSignal();

protected:
    std::string m_path;

    double m_scaleFactor;

    std::shared_ptr< slam::World > m_system;

    virtual void run() override;

private:
    void initialize();

};
