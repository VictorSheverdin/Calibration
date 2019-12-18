#pragma once

#include <opencv2/opencv.hpp>

#include "plane.h"

class ProjectionMatrix
{
public:
    ProjectionMatrix();
    ProjectionMatrix( const cv::Mat &projectionMatrix );

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2f &value );

    void setRotation( const cv::Mat &value );
    const cv::Mat &rotation() const;

    void setTranslation( const cv::Mat &value );
    const cv::Mat &translation() const;

    void setProjectionMatrix( const cv::Mat &value );
    const cv::Mat &projectionMatrix() const;

    Plane plane() const;

    operator cv::Mat() const;

private:
    cv::Mat m_cameraMatrix;

    cv::Mat m_r;
    cv::Mat m_t;

    mutable cv::Mat m_projectionMatrix;

    void initialize();

};
