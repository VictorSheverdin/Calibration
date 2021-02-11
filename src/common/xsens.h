#pragma once

#include <xsensdeviceapi.h>

#include <chrono>
#include <mutex>
#include <condition_variable>

#include <Eigen/Eigen>

#include "limitedqueue.h"

class XsensData : public XsDataPacket
{
public:
    XsensData();
    XsensData( const std::chrono::time_point< std::chrono::high_resolution_clock > &time, const XsDataPacket &packet );

    void setValues( const XsDataPacket &packet );

    void setUtcTime( const std::chrono::time_point< std::chrono::high_resolution_clock > &value );
    const std::chrono::time_point< std::chrono::high_resolution_clock > &utcTime() const;

    double xsensTime() const;

    const Eigen::Vector3d &acceleration() const;
    const Eigen::Vector3d &gyro() const;
    const Eigen::Vector3d &magnitometer() const;

    bool valid() const;

protected:
    std::chrono::time_point< std::chrono::high_resolution_clock > _time ;

    double _xsensTime;

    Eigen::Vector3d _acceleration;
    Eigen::Vector3d _gyro;
    Eigen::Vector3d _magnitometer;

    bool _valid;

private:

};

class XsensCallback : public XsCallback
{
public:
    XsensCallback();

    XsensData next( const std::chrono::milliseconds &timeout );
    std::vector< XsensData > all( const std::chrono::milliseconds &timeout );

protected:
    void onLiveDataAvailable( XsDevice *, const XsDataPacket *packet ) override;

private:
    std::mutex _mutex;
    std::condition_variable _condition;
    LimitedQueue< XsensData > _buffer;

    static const size_t m_bufferSize = 1000;
};

struct XsControl;
struct XsDevice;

class XsensInterface
{
public:
    XsensInterface();
    ~XsensInterface();

    XsensData getPacket( std::chrono::milliseconds timeout );
    std::vector< XsensData > getAllPackets( const std::chrono::milliseconds &timeout );

    bool connectDevice();
    bool connectDevice( const std::string &portName );

    bool prepare();
    void close();

private:
    XsControl *_control;
    XsDevice *_device;
    XsPortInfo _port;
    XsensCallback _xsensCallback;

};
