#include "Settings.h"

namespace ORB_SLAM2
{

Settings::Settings(
        const float imageWidth,
        const float imageHeight,
        const float fx,
        const float fy,
        const float cx,
        const float cy,
        const float k1,
        const float k2,
        const float k3,
        const float p1,
        const float p2,
        const float bf )
{
    initialize();

    setImageWidth( imageWidth );
    setImageHeight( imageHeight );
    setFx( fx );
    setFy( fy );
    setCx( cx );
    setCy( cy );
    setK1( k1 );
    setK2( k2 );
    setK3( k3 );
    setP1( p1 );
    setP2( p2 );
    setBf( bf );
}

Settings::Settings( const MonocularCalibrationDataShort &data )
{
    initialize();

    setImageWidth( data.frameWidth() );
    setImageHeight( data.frameHeight() );
    setFx( data.fx() );
    setFy( data.fy() );
    setCx( data.cx() );
    setCy( data.cy() );
    setK1( data.k1() );
    setK2( data.k2() );
    setK3( data.k3() );
    setP1( data.p1() );
    setP2( data.p2() );

    setBf( 0 );
}

Settings::Settings( const StereoCalibrationDataShort &data )
{
    initialize();

    setImageWidth( data.leftCameraResults().frameWidth() );
    setImageHeight( data.leftCameraResults().frameHeight() );
    setFx( data.leftCameraResults().fx() );
    setFy( data.leftCameraResults().fy() );
    setCx( data.leftCameraResults().cx() );
    setCy( data.leftCameraResults().cy() );
    setK1( data.leftCameraResults().k1() );
    setK2( data.leftCameraResults().k2() );
    setK3( data.leftCameraResults().k3() );
    setP1( data.leftCameraResults().p1() );
    setP2( data.leftCameraResults().p2() );
    setBf( data.distance() * data.leftCameraResults().fx() );
}

void Settings::initialize()
{
    m_keyFrameSize = 0.6;
    m_keyFrameLineWidth = 2;
    m_graphLineWidth = 1;
    m_pointSize = 2;
    m_cameraSize = 0.7;
    m_cameraLineWidth = 3;

    m_fps = 10;

    m_rgb = 1;

    m_features = 8000;
    m_scaleFactor = 1.2;
    m_levels = 24;
    m_iniThFAST = 77;
    m_minThFAST = 77;
    m_thDepth = 35;

    m_depthMapFactor = 1;

    m_viewpointX = 0;
    m_viewpointY = -100;
    m_viewpointZ = -0.1;
    m_viewpointF = 500;

}

void Settings::setkeyFrameSize( const float value )
{
    m_keyFrameSize = value;
}

float Settings::keyFrameSize() const
{
    return m_keyFrameSize;
}

void Settings::setKeyFrameLineWidth( const float value )
{
    m_keyFrameLineWidth = value;
}

float Settings::keyFrameLineWidth() const
{
    return m_keyFrameLineWidth;
}

void Settings::setGraphLineWidth( const float value )
{
    m_graphLineWidth = value;
}

float Settings::graphLineWidth() const
{
    return m_graphLineWidth;
}

void Settings::setPointSize( const float value )
{
    m_pointSize = value;
}

float Settings::pointSize() const
{
    return m_pointSize;
}

void Settings::setCameraSize( const float value )
{
    m_cameraSize = value;
}

float Settings::cameraSize() const
{
    return m_cameraSize;
}

void Settings::setCameraLineWidth( const float value )
{
    m_cameraLineWidth = value;
}

float Settings::cameraLineWidth() const
{
    return m_cameraLineWidth;
}

void Settings::setFx( const float value )
{
    m_fx = value;
}

float Settings::fx() const
{
    return m_fx;
}

void Settings::setFy( const float value )
{
    m_fy = value;
}

float Settings::fy() const
{
    return m_fy;
}

void Settings::setCx( const float value )
{
    m_cx = value;
}

float Settings::cx() const
{
    return m_cx;
}

void Settings::setCy( const float value )
{
    m_cy = value;
}

float Settings::cy() const
{
    return m_cy;
}

void Settings::setK1( const float value )
{
    m_k1 = value;
}

float Settings::k1() const
{
    return m_k1;
}

void Settings::setK2( const float value )
{
    m_k2 = value;
}

float Settings::k2() const
{
    return m_k2;
}

void Settings::setK3( const float value )
{
    m_k3 = value;
}

float Settings::k3() const
{
    return m_k3;
}

void Settings::setP1( const float value )
{
    m_p1 = value;
}

float Settings::p1() const
{
    return m_p1;
}

void Settings::setP2( const float value )
{
    m_p2 = value;
}

float Settings::p2() const
{
    return m_p2;
}

void Settings::setBf( const float value )
{
    m_bf = value;
}

float Settings::bf() const
{
    return m_bf;
}

void Settings::setFps( const float value )
{
    m_fps = value;
}

float Settings::fps() const
{
    return m_fps;
}

void Settings::setRgb( const int value )
{
    m_rgb = value;
}

int Settings::rgb() const
{
    return m_rgb;
}

void Settings::setFeatures( const int value )
{
    m_features = value;
}

int Settings::features() const
{
    return m_features;
}

void Settings::setScaleFactor( const float value )
{
    m_scaleFactor = value;
}

float Settings::scaleFactor() const
{
    return m_scaleFactor;
}

void Settings::setLevels( const int value )
{
    m_levels = value;
}

int Settings::levels() const
{
    return m_levels;
}

void Settings::setIniThFAST( const int value )
{
    m_iniThFAST = value;
}

int Settings::iniThFAST() const
{
    return m_iniThFAST;
}

void Settings::setMinThFAST( const int value )
{
    m_minThFAST = value;
}

int Settings::minThFAST() const
{
    return m_minThFAST;
}

void Settings::setThDepth( const float value )
{
    m_thDepth = value;
}

float Settings::thDepth() const
{
    return m_thDepth;
}

void Settings::setDepthMapFactor( const float value )
{
    m_depthMapFactor = value;
}

float Settings::depthMapFactor() const
{
    return m_depthMapFactor;
}

void Settings::setImageWidth( const float value )
{
    m_imageWidth = value;
}

float Settings::imageWidth() const
{
    return m_imageWidth;
}

void Settings::setImageHeight( const float value )
{
    m_imageHeight = value;
}

float Settings::imageHeight() const
{
    return m_imageHeight;
}

void Settings::setViewpointX( const float value )
{
    m_viewpointX = value;
}

float Settings::viewpointX() const
{
    return m_viewpointX;
}

void Settings::setViewpointY( const float value )
{
    m_viewpointY = value;
}

float Settings::viewpointY() const
{
    return m_viewpointY;
}

void Settings::setViewpointZ( const float value )
{
    m_viewpointZ = value;
}

float Settings::viewpointZ() const
{
    return m_viewpointZ;
}

void Settings::setViewpointF( const float value )
{
    m_viewpointF = value;
}

float Settings::viewpointF() const
{
    return m_viewpointF;
}

void Settings::zeroDistortionCoefficients()
{
    m_k1 = 0;
    m_k2 = 0;
    m_k3 = 0;
    m_p1 = 0;
    m_p2 = 0;
}

}
