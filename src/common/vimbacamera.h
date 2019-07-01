#pragma once

#include <QObject>

#include <QMutex>

#include "limitedqueue.h"

#include "VimbaCPP/Include/VimbaCPP.h"

#include "defs.h"
#include "image.h"

class FrameObserver : public QObject, public AVT::VmbAPI::IFrameObserver
{
    Q_OBJECT

public :
    FrameObserver( AVT::VmbAPI::CameraPtr pCamera );

    virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame ) override;

    Frame getFrame();
    Frame nearestFrame( const Frame &frame );

signals:
    void receivedFrame();

protected:
    QMutex m_framesMutex;
    LimitedQueue< Frame > m_framesQueue;
    int m_number;


    static int m_currentNumber;

private:
    void inititalize();

};

class CameraBase : public QObject
{
    Q_OBJECT

public:
    CameraBase( QObject *parent = nullptr );

    Frame getFrame();
    Frame nearestFrame( const Frame &frame );

signals:
    void receivedFrame();

protected:
    AVT::VmbAPI::CameraPtr m_camera;
    SP_DECL( FrameObserver ) m_frameObserver;

    void setMaxValue(const char * const name );

    static const int m_numFrames = 3;

private:
    void initialize();

};


class MasterCamera : public CameraBase
{
    Q_OBJECT

public:
    MasterCamera( const std::string &ip, QObject *parent = nullptr );
    ~MasterCamera();

private:
    void initialize( const std::string &ip );

};

class SlaveCamera : public CameraBase
{
    Q_OBJECT

public:
    SlaveCamera( const std::string &ip, QObject *parent = nullptr );
    ~SlaveCamera();

private:
    void initialize( const std::string &ip );

};

class StereoCamera : public QObject
{
    Q_OBJECT

public:
    StereoCamera( const std::string &leftIp, const std::string &rightIp, QObject *parent = nullptr );

    StereoFrame getFrame();

    bool empty() const;

signals:
    void receivedFrame();

protected slots:
    void updateFrame();

protected:
    MasterCamera m_leftCamera;
    SlaveCamera m_rightCamera;

    LimitedQueue< StereoFrame > m_framesQueue;
    QMutex m_framesMutex;

private:
    void initialize();

};
