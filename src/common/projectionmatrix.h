#pragma once

#include <opencv2/opencv.hpp>

#include "plane.h"

#include <g2o/types/slam3d/se3quat.h>

cv::Mat calcProjectionMatrix( const cv::Mat &cameraMatrix, const cv::Mat &r, const cv::Mat &t );

class ProjectionMatrix
{
public:
    ProjectionMatrix();
    ProjectionMatrix( const cv::Mat &cameraMatrix, const cv::Mat &rotationMatrix, const cv::Mat &translationMatrix );
    ProjectionMatrix( const ProjectionMatrix &other );
    ProjectionMatrix( const cv::Mat &projectionMatrix );
    ProjectionMatrix( const std::string &fileName );

    void setCameraMatrix( const cv::Mat &value );
    const cv::Mat &cameraMatrix() const;

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2d &value );

    void rotate( const cv::Mat &mat );
    void translate( const cv::Mat &vec );

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

    Plane zeroPlane() const;
    Plane plane() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

    operator cv::Mat() const;

    bool operator==( const ProjectionMatrix &other ) const;
    ProjectionMatrix &operator=( const ProjectionMatrix &other );

    void setSe3Pose( const g2o::SE3Quat &pose );
    g2o::SE3Quat se3Pose() const;

    operator cv::Point3d() const;

private:
    cv::Mat m_cameraMatrix;

    cv::Mat m_r;
    cv::Mat m_t;

    mutable cv::Mat m_projectionMatrix;

    void initialize();

};

std::ostream &operator<<( std::ostream& out, const ProjectionMatrix& matrix );

class StereoProjectionMatrix
{
    friend std::ostream &operator<<( std::ostream& out, const StereoProjectionMatrix& matrix );

public:

    StereoProjectionMatrix();
    StereoProjectionMatrix( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix );
    StereoProjectionMatrix( const cv::Mat &leftProjectionMatrix, const cv::Mat &rightProjectionMatrix );
    StereoProjectionMatrix( const std::string &fileName );

    void setLeftProjectionMatrix( const cv::Mat &value );
    void setLeftProjectionMatrix( const ProjectionMatrix &value );

    void setRightProjectionMatrix( const cv::Mat &value );
    void setRightProjectionMatrix( const ProjectionMatrix &value );

    void multiplicateCameraMatrix( const double value );
    void movePrincipalPoint( const cv::Vec2f &value );

    void rotate( const cv::Mat &mat );
    void translate( const cv::Mat &vec );

    const ProjectionMatrix &leftProjectionMatrix() const;
    const ProjectionMatrix &rightProjectionMatrix() const;

    cv::Mat baselineVector() const;
    double baselineVectorLenght() const;

    cv::Mat disparityToDepthMatrix() const;

    bool saveYaml( const std::string &fileName ) const;
    bool loadYaml( const std::string &fileName );

    bool operator==( const StereoProjectionMatrix &other ) const;
    bool operator!=( const StereoProjectionMatrix &other ) const;

protected:
    ProjectionMatrix m_leftProjectionMatrix;
    ProjectionMatrix m_rightProjectionMatrix;

};

std::ostream &operator<<( std::ostream& out, const StereoProjectionMatrix& matrix );
