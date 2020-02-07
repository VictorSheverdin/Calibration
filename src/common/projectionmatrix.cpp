#include "precompiled.h"

#include "projectionmatrix.h"

#include <opencv2/sfm.hpp>

#include "src/common/defs.h"

// ProjectionMatrix
ProjectionMatrix::ProjectionMatrix()
{
    initialize();
}

ProjectionMatrix::ProjectionMatrix( const cv::Mat &projectionMatrix )
{
    initialize();

    setProjectionMatrix( projectionMatrix );
}

ProjectionMatrix::ProjectionMatrix( const std::string &fileName )
{
    initialize();

    loadYaml( fileName );

}

void ProjectionMatrix::initialize()
{
    setCameraMatrix( cv::Mat::eye( 3, 3, CV_64F ) );

    setRotation( cv::Mat::eye( 3, 3, CV_64F ) );
    setTranslation( cv::Mat::zeros( 3, 1, CV_64F ) );
}

void ProjectionMatrix::setCameraMatrix( const cv::Mat &value )
{
    m_projectionMatrix = cv::Mat();

    m_cameraMatrix = value;
}

const cv::Mat &ProjectionMatrix::cameraMatrix() const
{
    return m_cameraMatrix;
}

void ProjectionMatrix::multiplicateCameraMatrix( const double value )
{
    m_projectionMatrix = cv::Mat();

    m_cameraMatrix.at< double >( 0, 0 ) *= value;
    m_cameraMatrix.at< double >( 1, 1 ) *= value;
    m_cameraMatrix.at< double >( 0, 2 ) *= value;
    m_cameraMatrix.at< double >( 1, 2 ) *= value;

}

void ProjectionMatrix::movePrincipalPoint(const cv::Vec2d &value )
{
    m_projectionMatrix = cv::Mat();

    m_cameraMatrix.at< double >( 0, 2 ) += value[ 0 ];
    m_cameraMatrix.at< double >( 1, 2 ) += value[ 1 ];
}

void ProjectionMatrix::setRotation( const cv::Mat &value )
{
    m_projectionMatrix = cv::Mat();

    m_r = value;
}

const cv::Mat &ProjectionMatrix::rotation() const
{
    return m_r;
}

void ProjectionMatrix::setTranslation( const cv::Mat &value )
{
    m_projectionMatrix = cv::Mat();

    m_t = value;
}

const cv::Mat &ProjectionMatrix::translation() const
{
    return m_t;
}

cv::Vec3d ProjectionMatrix::translationVector() const
{
    return cv::Vec3d( x(), y(), z() );
}

void ProjectionMatrix::setProjectionMatrix( const cv::Mat &value )
{
    m_projectionMatrix = value;

    cv::sfm::KRtFromProjection( m_projectionMatrix, m_cameraMatrix, m_r, m_t );

}

const cv::Mat &ProjectionMatrix::projectionMatrix() const
{
    if ( m_projectionMatrix.empty() ) {
        auto extrinsicMatrix = cv::Mat( 3, 4, CV_64F );
        m_r.copyTo( extrinsicMatrix.rowRange( 0, 3 ).colRange( 0, 3 ) );
        m_t.copyTo( extrinsicMatrix.rowRange( 0, 3 ).col( 3 ) );
        m_projectionMatrix = m_cameraMatrix * extrinsicMatrix;

    }

    return m_projectionMatrix;

}

double ProjectionMatrix::x() const
{
    return m_t.at< double >( 0, 0 );
}

double ProjectionMatrix::y() const
{
    return m_t.at< double >( 1, 0 );
}

double ProjectionMatrix::z() const
{
    return m_t.at< double >( 2, 0 );
}

double ProjectionMatrix::fx() const
{
    return m_cameraMatrix.at< double >( 0, 0 );
}

double ProjectionMatrix::fy() const
{
    return m_cameraMatrix.at< double >( 1, 1 );
}

double ProjectionMatrix::cx() const
{
    return m_cameraMatrix.at< double >( 0, 2 );
}

double ProjectionMatrix::cy() const
{
    return m_cameraMatrix.at< double >( 1, 2 );
}

Plane ProjectionMatrix::plane() const
{
    auto a = m_r.at< double >( 0, 2 );
    auto b = m_r.at< double >( 1, 2 );
    auto c = m_r.at< double >( 2, 2 );

    return Plane( a, b, c, -a * x() - b * y() - c * z() );

}

bool ProjectionMatrix::saveYaml( const std::string &fileName ) const
{
    cv::FileStorage fs( fileName, cv::FileStorage::WRITE );
    if ( !fs.isOpened() )
        return false;

    time_t rawtime; time( &rawtime );
    fs << "savingDate" << asctime( localtime( &rawtime ) );

    fs << "cameraMatrix" << cameraMatrix();
    fs << "rotationMatrix" << rotation();
    fs << "translationVector" << translation();

    fs << "projectionMatrix" << projectionMatrix();

    fs.release();

    return true;
}

bool ProjectionMatrix::loadYaml( const std::string &fileName )
{
    cv::FileStorage fs ( fileName, cv::FileStorage::READ );

    if ( !fs.isOpened() )
        return false;

    cv::Mat projectionMatrix;
    fs["projectionMatrix"] >> projectionMatrix;

    if ( !projectionMatrix.empty() ) {
        setProjectionMatrix( projectionMatrix );
        return true;
    }

    cv::Mat cameraMatrix;
    cv::Mat rotationMatrix;
    cv::Mat translationVector;

    fs["cameraMatrix"] >> cameraMatrix;
    fs["rotationMatrix"] >> rotationMatrix;
    fs["translationVector"] >> translationVector;

    if ( !cameraMatrix.empty() && !rotationMatrix.empty() && !translationVector.empty() ) {
        setCameraMatrix( cameraMatrix );
        setRotation( rotationMatrix );
        setTranslation( translationVector );

        return true;
    }

    return false;

}

ProjectionMatrix::operator cv::Mat() const
{
    return projectionMatrix();
}

// StereoCameraMatrix
StereoCameraMatrix::StereoCameraMatrix()
{
}

StereoCameraMatrix::StereoCameraMatrix( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
    : m_leftProjectionMatrix( leftProjectionMatrix ), m_rightProjectionMatrix( rightProjectionMatrix )
{
}

StereoCameraMatrix::StereoCameraMatrix( const cv::Mat &leftProjectionMatrix, const cv::Mat &rightProjectionMatrix )
    : m_leftProjectionMatrix( leftProjectionMatrix ), m_rightProjectionMatrix( rightProjectionMatrix )
{
}

StereoCameraMatrix::StereoCameraMatrix( const std::string &fileName )
{
    loadYaml( fileName );
}

void StereoCameraMatrix::setLeftProjectionMatrix( const cv::Mat &value )
{
    m_leftProjectionMatrix.setProjectionMatrix( value );
}

void StereoCameraMatrix::setLeftProjectionMatrix( const ProjectionMatrix &value )
{
    m_leftProjectionMatrix = value;
}

void StereoCameraMatrix::setRightProjectionMatrix( const cv::Mat &value )
{
    m_rightProjectionMatrix.setProjectionMatrix( value );
}

void StereoCameraMatrix::setRightProjectionMatrix( const ProjectionMatrix &value )
{
    m_rightProjectionMatrix = value;
}

void StereoCameraMatrix::multiplicateCameraMatrix( const double value )
{
    m_leftProjectionMatrix.multiplicateCameraMatrix( value );
    m_rightProjectionMatrix.multiplicateCameraMatrix( value );
}

void StereoCameraMatrix::movePrincipalPoint( const cv::Vec2f &value )
{
    m_leftProjectionMatrix.movePrincipalPoint( value );
    m_rightProjectionMatrix.movePrincipalPoint( value );
}

const ProjectionMatrix &StereoCameraMatrix::leftProjectionMatrix() const
{
    return m_leftProjectionMatrix;
}

const ProjectionMatrix &StereoCameraMatrix::rightProjectionMatrix() const
{
    return m_rightProjectionMatrix;
}

cv::Mat StereoCameraMatrix::baselineVector() const
{
    return m_rightProjectionMatrix.translation() - m_leftProjectionMatrix.translation();
}

double StereoCameraMatrix::baselineVectorLenght() const
{
    return cv::norm( baselineVector() );
}

cv::Mat StereoCameraMatrix::disparityToDepthMatrix() const
{
    auto tx = baselineVectorLenght();

    if ( tx > DOUBLE_EPS ) {

        auto cx = leftProjectionMatrix().cx();
        auto cx0 = rightProjectionMatrix().cx();
        auto cy = leftProjectionMatrix().cy();
        auto f = leftProjectionMatrix().fx();

        return cv::Mat_< double >( 4, 4 ) <<
                                            1, 0, 0, -cx,
                                            0, 1, 0, -cy,
                                            0, 0, 0, f,
                                            0, 0, 1.0 / tx, ( cx - cx0 ) / tx ;

    }

    return cv::Mat();

}

bool StereoCameraMatrix::saveYaml( const std::string &fileName ) const
{
    cv::FileStorage fs( fileName, cv::FileStorage::WRITE );
    if ( !fs.isOpened() )
        return false;

    time_t rawtime; time( &rawtime );
    fs << "savingDate" << asctime( localtime( &rawtime ) );

    fs << "leftCameraMatrix" << m_leftProjectionMatrix.cameraMatrix();
    fs << "leftRotationMatrix" << m_leftProjectionMatrix.rotation();
    fs << "leftTranslationVector" << m_leftProjectionMatrix.translation();
    fs << "leftProjectionMatrix" << m_leftProjectionMatrix.projectionMatrix();

    fs << "rightCameraMatrix" << m_rightProjectionMatrix.cameraMatrix();
    fs << "rightRotationMatrix" << m_rightProjectionMatrix.rotation();
    fs << "rightTranslationVector" << m_rightProjectionMatrix.translation();
    fs << "rightProjectionMatrix" << m_rightProjectionMatrix.projectionMatrix();

    fs.release();

    return true;
}

bool StereoCameraMatrix::loadYaml( const std::string &fileName )
{
    cv::FileStorage fs ( fileName, cv::FileStorage::READ );

    if ( !fs.isOpened() )
        return false;

    cv::Mat leftProjectionMatrix;
    cv::Mat rightProjectionMatrix;
    fs["leftProjectionMatrix"] >> leftProjectionMatrix;
    fs["rightProjectionMatrix"] >> rightProjectionMatrix;

    if ( !leftProjectionMatrix.empty() && !rightProjectionMatrix.empty() ) {
        m_leftProjectionMatrix.setProjectionMatrix( leftProjectionMatrix );
        m_rightProjectionMatrix.setProjectionMatrix( rightProjectionMatrix );

        return true;

    }

    cv::Mat leftCameraMatrix;
    cv::Mat leftRotationMatrix;
    cv::Mat leftTranslationVector;

    cv::Mat rightCameraMatrix;
    cv::Mat rightRotationMatrix;
    cv::Mat rightTranslationVector;

    fs["leftCameraMatrix"] >> leftCameraMatrix;
    fs["leftRotationMatrix"] >> leftRotationMatrix;
    fs["leftTranslationVector"] >> leftTranslationVector;

    fs["rightCameraMatrix"] >> rightCameraMatrix;
    fs["rightRotationMatrix"] >> rightRotationMatrix;
    fs["rightTranslationVector"] >> rightTranslationVector;

    if ( !leftCameraMatrix.empty() && !leftRotationMatrix.empty() && !leftTranslationVector.empty() &&
            !rightCameraMatrix.empty() && !rightRotationMatrix.empty() && !rightTranslationVector.empty() ) {

        m_leftProjectionMatrix.setCameraMatrix( leftCameraMatrix );
        m_leftProjectionMatrix.setRotation( leftRotationMatrix );
        m_leftProjectionMatrix.setTranslation( leftTranslationVector );

        m_rightProjectionMatrix.setCameraMatrix( rightCameraMatrix );
        m_rightProjectionMatrix.setRotation( rightRotationMatrix );
        m_rightProjectionMatrix.setTranslation( rightTranslationVector );

        return true;

    }

    return false;
}
