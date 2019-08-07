#pragma once

#include "src/common/calibrationdatabase.h"

namespace ORB_SLAM2
{

class Settings
{
public:
    Settings( const float imageWidth, const float imageHeight, const float fx, const float fy, const float cx, const float cy,
                    const float k1, const float k2, const float k3, const float p1, const float p2, const float bf );

    Settings( const MonocularCalibrationDataShort &data );
    Settings( const StereoCalibrationDataShort &data );

    // MapDrawer settings
    void setkeyFrameSize( const float value );
    float keyFrameSize() const;
    void setKeyFrameLineWidth( const float value );
    float keyFrameLineWidth() const;
    void setGraphLineWidth( const float value );
    float graphLineWidth() const;
    void setPointSize( const float value );
    float pointSize() const;
    void setCameraSize( const float value );
    float cameraSize() const;
    void setCameraLineWidth( const float value );
    float cameraLineWidth() const;

    // Tracking settings
    void setFx( const float value );
    float fx() const;
    void setFy( const float value );
    float fy() const;
    void setCx( const float value );
    float cx() const;
    void setCy( const float value );
    float cy() const;

    void setK1( const float value );
    float k1() const;
    void setK2( const float value );
    float k2() const;
    void setK3( const float value );
    float k3() const;
    void setP1( const float value );
    float p1() const;
    void setP2( const float value );
    float p2() const;

    void setBf( const float value );
    float bf() const;

    void setFps( const float value );
    float fps() const;

    void setRgb( const int value );
    int rgb() const;

    void setFeatures( const int value );
    int features() const;
    void setScaleFactor( const float value );
    float scaleFactor() const;

    void setLevels( const int value );
    int levels() const;
    void setIniThFAST( const int value );
    int iniThFAST() const;
    void setMinThFAST( const int value );
    int minThFAST() const;

    void setThDepth( const float value );
    float thDepth();

    void setDepthMapFactor( const float value );
    float depthMapFactor();

    // Viewer settings
    void setImageWidth( const float value );
    float imageWidth() const;
    void setImageHeight( const float value );
    float imageHeight() const;

    void setViewpointX( const float value );
    float viewpointX() const;
    void setViewpointY( const float value );
    float viewpointY() const;
    void setViewpointZ( const float value );
    float viewpointZ() const;
    void setViewpointF( const float value );
    float viewpointF() const;

protected:
    float m_keyFrameSize;
    float m_keyFrameLineWidth;
    float m_graphLineWidth;
    float m_pointSize;
    float m_cameraSize;
    float m_cameraLineWidth;

    float m_fx;
    float m_fy;
    float m_cx;
    float m_cy;

    float m_k1;
    float m_k2;
    float m_k3;
    float m_p1;
    float m_p2;

    float m_bf;

    float m_fps;

    int m_rgb;

    int m_features;
    float m_scaleFactor;
    int m_levels;
    int m_iniThFAST;
    int m_minThFAST;

    float m_thDepth;

    float m_depthMapFactor;

    float m_imageWidth;
    float m_imageHeight;

    float m_viewpointX;
    float m_viewpointY;
    float m_viewpointZ;
    float m_viewpointF;

private:
    void initialize();

};

}
