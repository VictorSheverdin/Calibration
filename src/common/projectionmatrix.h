#pragma once

#include <opencv2/opencv.hpp>

#include "plane.h"

class ProjectionMatrix
{
public:
    ProjectionMatrix();
    ProjectionMatrix( const cv::Mat &projectionMatrix );
    ProjectionMatrix( const std::string &fileName );

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2d &value );

    void setRotation( const cv::Mat &value );
    const cv::Mat &rotation() const;

    void setTranslation( const cv::Mat &value );
    const cv::Mat &translation() const;
    cv::Vec3d translationVector() const;

    void setProjectionMatrix( const cv::Mat &value );
    const cv::Mat &projectionMatrix() const;

    double x() const;
    double y() const;
    double z() const;

    double fx() const;
    double fy() const;
    double cx() const;
    double cy() const;

    Plane plane() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

    operator cv::Mat() const;

private:
    cv::Mat m_cameraMatrix;

    cv::Mat m_r;
    cv::Mat m_t;

    mutable cv::Mat m_projectionMatrix;

    void initialize();

};

class StereoCameraMatrix
{
public:
    StereoCameraMatrix();
    StereoCameraMatrix( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );
    StereoCameraMatrix( const cv::Mat &leftProjectionMatrix, const cv::Mat &rightProjectionMatrix );
    StereoCameraMatrix( const std::string &fileName );

    void setLeftProjectionMatrix( const cv::Mat &value );
    void setLeftProjectionMatrix( const ProjectionMatrix &value );

    void setRightProjectionMatrix( const cv::Mat &value );
    void setRightProjectionMatrix( const ProjectionMatrix &value );

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2f &value );

    const ProjectionMatrix &leftProjectionMatrix() const;
    const ProjectionMatrix &rightProjectionMatrix() const;

    const cv::Mat baselineVector() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

protected:
    ProjectionMatrix m_leftProjectionMatrix;
    ProjectionMatrix m_rightProjectionMatrix;

};
