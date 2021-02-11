#include "precompiled.h"

#include "xsens.h"

Journaller *gJournal = 0;

// XsensData
XsensData::XsensData()
    : _valid( false )
{
    setUtcTime( std::chrono::high_resolution_clock::now() );
}

XsensData::XsensData( const std::chrono::time_point< std::chrono::high_resolution_clock > &time, const XsDataPacket &packet )
    : XsDataPacket( packet )
{
    setUtcTime( time );

    setValues( packet );
}

void XsensData::setValues( const XsDataPacket &packet )
{
    _valid = false;

    if ( packet.containsSampleTimeFine() ) {

        _xsensTime = packet.sampleTimeFine() * 1.e-4;

        if ( packet.containsCalibratedData() ) {

            auto accelData = packet.calibratedAcceleration();
            auto gyroData = packet.calibratedGyroscopeData();
            auto magData = packet.calibratedMagneticField();

            _acceleration = Eigen::Vector3d( accelData[ 0 ], accelData[ 1 ], accelData[ 2 ] );
            _gyro = Eigen::Vector3d( gyroData[ 0 ], gyroData[ 1 ], gyroData[ 2 ] );
            _magnitometer = Eigen::Vector3d( magData[ 0 ], magData[ 1 ], magData[ 2 ] );

            _valid = true;

        }

    }

}

void XsensData::setUtcTime( const std::chrono::time_point< std::chrono::high_resolution_clock > &value )
{
    _time = value;
}

const std::chrono::time_point< std::chrono::high_resolution_clock > &XsensData::utcTime() const
{
    return _time;
}

double XsensData::xsensTime() const
{
    return _xsensTime;
}

const Eigen::Vector3d &XsensData::acceleration() const
{
    return _acceleration;
}

const Eigen::Vector3d &XsensData::gyro() const
{
    return _gyro;
}

const Eigen::Vector3d &XsensData::magnitometer() const
{
    return _magnitometer;
}

bool XsensData::valid() const
{
    return _valid;
}

// XsensCallback
XsensCallback::XsensCallback()
    : _buffer( m_bufferSize )
{
}

XsensData XsensCallback::next( const std::chrono::milliseconds &timeout )
{
    XsensData packet;

    std::unique_lock< std::mutex > lock( _mutex );

    if ( _condition.wait_for( lock, timeout, [ & ] { return !_buffer.empty(); } ) )
    {
        if ( !_buffer.empty() ) {

            packet = _buffer.front();
            _buffer.pop();

        }

    }

    return packet;

}

std::vector< XsensData > XsensCallback::all( const std::chrono::milliseconds &timeout )
{
    std::vector< XsensData > ret;

    std::unique_lock< std::mutex > lock( _mutex );

    if ( _condition.wait_for( lock, timeout, [ & ] { return !_buffer.empty(); } ) )
    {
        ret.reserve( _buffer.size() );

        for ( auto &i : _buffer )
            ret.push_back( i );

        _buffer.clear();

    }

    return ret;
}

void XsensCallback::onLiveDataAvailable( XsDevice *, const XsDataPacket *packet )
{
    auto now = std::chrono::high_resolution_clock::now();

    std::unique_lock< std::mutex > lock( _mutex );

    if( packet ) {

        _buffer.push( XsensData( now, *packet ) );

        lock.unlock();

        _condition.notify_one();

    }

}

// XsensInterface
XsensInterface::XsensInterface()
    : _device( nullptr )
{
    _control = XsControl::construct();
}

XsensInterface::~XsensInterface()
{
    close();

    if ( _control )
        _control->destruct();
}

XsensData XsensInterface::getPacket( std::chrono::milliseconds timeout )
{
    return _xsensCallback.next( timeout );
}

std::vector< XsensData > XsensInterface::getAllPackets( const std::chrono::milliseconds &timeout )
{
    return _xsensCallback.all( timeout );
}

bool XsensInterface::connectDevice()
{
    XsPortInfo mtPort;

    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

    for ( auto const &portInfo : portInfoArray ) {
        if ( portInfo.deviceId().isMti() || portInfo.deviceId().isMtig() ) {
            mtPort = portInfo;
            break;

        }

    }

    if ( ! mtPort.empty() ) {

        if ( _control->openPort( mtPort.portName().toStdString(), mtPort.baudrate() ) ) {

            _device = _control->device( mtPort.deviceId() );

            if ( _device ) {

                _device->addCallbackHandler( &_xsensCallback );
                return true;

            }

        }

    }

    return false;

}

#define XS_DEFAULT_BAUDRATE (115200)

bool XsensInterface::connectDevice( const std::string &portName )
{
    XsPortInfo mtPort;

    int baudrate = XS_DEFAULT_BAUDRATE;

    mtPort = XsPortInfo( portName, XsBaud_numericToRate( baudrate ) );

    if ( ! mtPort.empty() ) {

        if ( _control->openPort( mtPort.portName().toStdString(), mtPort.baudrate() ) ) {

            _device = _control->device( mtPort.deviceId() );

            if ( _device ) {

                _device->addCallbackHandler( &_xsensCallback );
                return true;

            }

        }

    }

    return false;

}

bool XsensInterface::prepare()
{
    if( _device ) {

        if ( _device->gotoConfig() ) {

            XsOutputConfigurationArray configArray;
            configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
            configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

            if (_device->deviceId().isImu())
            {
                configArray.push_back(XsOutputConfiguration(XDI_DeltaV, 0));
                configArray.push_back(XsOutputConfiguration(XDI_DeltaQ, 0));
                configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0));
            }
            else if (_device->deviceId().isVru() || _device->deviceId().isAhrs())
            {
                configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
            }
            else if (_device->deviceId().isGnss())
            {
                configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
                configArray.push_back(XsOutputConfiguration(XDI_LatLon, 0));
                configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 0));
                configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 0));
                configArray.push_back(XsOutputConfiguration(XDI_GnssPvtData, 0));
                configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 0));
                configArray.push_back(XsOutputConfiguration(XDI_BaroPressure, 0));
                configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0));
                configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 0));
                //configArray.push_back(XsOutputConfiguration(XDI_RawAccGyrMagTemp, 0));
            }
            else
                throw std::exception();

            if ( !_device->setOutputConfiguration( configArray ) )
                throw std::exception();

            if ( !_device->gotoMeasurement() )
                throw std::exception();

/*            if ( _device->readEmtsAndDeviceConfiguration() ) {

                if ( _device->gotoMeasurement() )
                    return true;

            }*/

        }

    }

    return false;

}

void XsensInterface::close()
{
    if ( _device )
    {
        _device->stopRecording();
        _device->closeLogFile();
        _device->removeCallbackHandler( &_xsensCallback );
    }

    _control->closePort( _port );

}
