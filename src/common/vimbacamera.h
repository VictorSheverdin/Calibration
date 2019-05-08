#pragma once

#include <QObject>
#include <QMutex>

#include <deque>

#include "VimbaCPP/Include/VimbaCPP.h"

#include "defs.h"
#include "image.h"

class FrameObserver : public QObject, public AVT::VmbAPI::IFrameObserver
{
    Q_OBJECT

public :
    FrameObserver( AVT::VmbAPI::CameraPtr pCamera );

    virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame ) override;

    CvImage getFrame();

protected:
    std::deque< CvImage > m_frames;
    QMutex m_framesMutex;

    static const int m_maxDequeSize = 10;

signals:
    void receivedFrame();

};

class VimbaCamera : public QObject
{
    Q_OBJECT

public:
    VimbaCamera( const std::string &ip );
    ~VimbaCamera();

    CvImage getFrame();

signals:
    void receivedFrame();

protected:
    AVT::VmbAPI::CameraPtr m_camera;
    SP_DECL( FrameObserver ) m_frameObserver;

    void setMaxValue(const char * const name );

    static const int m_numFrames = 3;

private:
    void initialize( const std::string &ip );
};
