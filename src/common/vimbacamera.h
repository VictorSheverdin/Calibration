#pragma once

#include <QObject>

#include <QMutex>

#include "limitedqueue.h"

#include <VimbaCPP/Include/VimbaCPP.h>

#include "defs.h"
#include "image.h"

static const int VIMBA_ORIGINAL_FRAME_SIZE = 2048;

enum class VimbaDecimationType { WHOLE = 1, HALF = 2, QUARTER = 4, EIGHTH = 8 };

static const VmbInt64_t ACTION_DEVICE_KEY = 1;
static const VmbInt64_t ACTION_GROUP_KEY = 1;
static const VmbInt64_t ACTION_GROUP_MASK = 1;

class FrameObserver : public QObject, public AVT::VmbAPI::IFrameObserver
{
    Q_OBJECT

    friend class CameraBase;

public :
    FrameObserver( AVT::VmbAPI::CameraPtr pCamera );

    virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame ) override;

    Frame getFrame();

signals:
    void receivedFrame();

protected:
    QMutex m_framesMutex;
    int m_number;

    cv::Mat m_sourceMat;
    std::chrono::time_point< std::chrono::system_clock > m_sourceTime;

    static int m_currentNumber;
    static const std::chrono::time_point< std::chrono::system_clock > m_startTime;

    static int64_t timeFromStart();

    void lockMutex();
    void unlockMutex();

    Frame getFrameUnsafe() const;
    std::chrono::time_point< std::chrono::system_clock > getTimeUnsafe() const;

private:
    void inititalize();

};

class CameraBase : public QObject
{
    Q_OBJECT

    friend class StereoCamera;

public:
    CameraBase( QObject *parent = nullptr );

    Frame getFrame();

signals:
    void receivedFrame();

protected:
    AVT::VmbAPI::CameraPtr m_camera;
    SP_DECL( FrameObserver ) m_frameObserver;

    static const int m_numFrames = 3;

    void setMaxValue(const char * const name );

    void lockMutex();
    void unlockMutex();

    Frame getFrameUnsafe() const;

    std::chrono::time_point< std::chrono::system_clock > getTimeUnsafe() const;

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

public slots:

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

void checkVimbaStatus( VmbErrorType status, std::string message );

template <typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );

}

template <typename FeatureT>
void setVimbaFeature( AVT::VmbAPI::VimbaSystem &system, const std::string &key, FeatureT value )
{
    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( system.GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->SetValue(value), "Could not set " + key );

}

template <typename FeatureT>
FeatureT getVimbaFeature( AVT::VmbAPI::CameraPtr camera, const std::string &key )
{
    FeatureT ret;

    AVT::VmbAPI::FeaturePtr feature;

    checkVimbaStatus( camera->GetFeatureByName( key.data(), feature ),
        "Could not access " + key);

    checkVimbaStatus( feature->GetValue( ret ), "Could not get " + key );

    return ret;

}

void vimbaRunCommand( AVT::VmbAPI::VimbaSystem &system, const std::string &key );

void vimbaRunCommand(AVT::VmbAPI::CameraPtr camera, const std::string &key );
