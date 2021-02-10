#include "precompiled.h"

#include "projectionmatrix.h"

#include <opencv2/sfm.hpp>

#include "defs.h"

cv::Mat calcProjectionMatrix( const cv::Mat &cameraMatrix, const cv::Mat &r, const cv::Mat &t )
{
    cv::Mat ret;

    // cv::sfm::projectionFromKRt( cameraMatrix, r, t, ret );

    auto extrinsicMatrix = cv::Mat( 3, 4, CV_64F );
    r.copyTo( extrinsicMatrix.rowRange( 0, 3 ).colRange( 0, 3 ) );
    t.copyTo( extrinsicMatrix.rowRange( 0, 3 ).col( 3 ) );
    ret = cameraMatrix * extrinsicMatrix;

    return ret;
}

// ProjectionMatrix
ProjectionMatrix::ProjectionMatrix()
{
    initialize();
}

ProjectionMatrix::ProjectionMatrix( const cv::Mat &cameraMatrix, const cv::Mat &rotationMatrix, const cv::Mat &translationMatrix )
{
    initialize();

    setCameraMatrix( cameraMatrix );
    setRotation( rotationMatrix );
    setTranslation( translationMatrix );
}


ProjectionMatrix::ProjectionMatrix( const ProjectionMatrix &other )
{
    m_projectionMatrix = cv::Mat();
    other.m_cameraMatrix.copyTo( m_cameraMatrix );
    other.m_r.copyTo( m_r );
    other.m_t.copyTo( m_t );
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

void ProjectionMatrix::rotate( const cv::Mat &mat )
{
    m_projectionMatrix = cv::Mat();

    m_r *= mat;

}

void ProjectionMatrix::translate( const cv::Mat &vec )
{
    m_projectionMatrix = cv::Mat();

    m_t += vec;
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
    if ( m_projectionMatrix.empty() )
        m_projectionMatrix = calcProjectionMatrix( m_cameraMatrix, m_r, m_t );

    return m_projectionMatrix;

}

double ProjectionMatrix::x() const
{
    return translation().at< double >( 0, 0 );
}

double ProjectionMatrix::y() const
{
    return translation().at< double >( 1, 0 );
}

double ProjectionMatrix::z() const
{
    return translation().at< double >( 2, 0 );
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

Plane ProjectionMatrix::zeroPlane() const
{
    cv::Mat rt = rotation().t();

    return Plane( rt.at< double >( 0, 2 ), rt.at< double >( 1, 2 ), rt.at< double >( 2, 2 ), 0.0 );

}

Plane ProjectionMatrix::plane() const
{
    auto ret = zeroPlane();

    ret.setD( ret.a() * x() + ret.b() * y() + ret.c() * z() );

    return ret;
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

bool ProjectionMatrix::operator==( const ProjectionMatrix &other ) const
{
    return std::abs( cv::norm( projectionMatrix() - other.projectionMatrix() ) ) <= DOUBLE_EPS ;
}

ProjectionMatrix &ProjectionMatrix::operator=( const ProjectionMatrix &other )
{
    if ( this != &other ) {
        m_projectionMatrix = cv::Mat();
        other.m_cameraMatrix.copyTo( m_cameraMatrix );
        other.m_r.copyTo( m_r );
        other.m_t.copyTo( m_t );
    }

    return *this;
}

g2o::SE3Quat ProjectionMatrix::se3Pose() const
{
    auto rMat = rotation();
    auto tMat = translation();

    Eigen::Matrix< double, 3, 3 > r;

    r << rMat.at< double >( 0, 0 ), rMat.at< double >( 0, 1 ), rMat.at< double >( 0, 2 ),
         rMat.at< double >( 1, 0 ), rMat.at< double >( 1, 1 ), rMat.at< double >( 1, 2 ),
         rMat.at< double >( 2, 0 ), rMat.at< double >( 2, 1 ), rMat.at< double >( 2, 2 );

    Eigen::Matrix< double, 3, 1 > t( tMat.at< double >( 0, 0 ), tMat.at< double >( 1, 0 ), tMat.at< double >( 2, 0 ) );

    return g2o::SE3Quat( r, t );

}

void ProjectionMatrix::setSe3Pose( const g2o::SE3Quat &pose )
{
    auto homogeniousMatrix = pose.to_homogeneous_matrix();

    auto rMat = rotation();
    auto tMat = translation();

    for ( auto i = 0; i < 3; ++i )
        for ( auto j = 0; j < 3; ++j )
            rMat.at< double >( i, j ) = homogeniousMatrix( i, j );

    for ( auto i = 0; i < 3; ++i )
            tMat.at< double >( i, 0 ) = homogeniousMatrix( i, 3 );

    setRotation( rMat );
    setTranslation( tMat );

}

ProjectionMatrix::operator cv::Point3d() const
{
    auto t = translation();

    return  cv::Point3d( t.at< double >( 0, 0 ), t.at< double >( 1, 0 ), t.at< double >( 2, 0 ) );
}

std::ostream &operator<<( std::ostream& out, const ProjectionMatrix& matrix )
{
    out << matrix.projectionMatrix() << std::endl;

    return out;
}

// StereoProjectionMatrix
StereoProjectionMatrix::StereoProjectionMatrix()
{
}

StereoProjectionMatrix::StereoProjectionMatrix( const ProjectionMatrix &leftProjectionMatrix, const ProjectionMatrix &rightProjectionMatrix )
    : m_leftProjectionMatrix( leftProjectionMatrix ), m_rightProjectionMatrix( rightProjectionMatrix )
{
}

StereoProjectionMatrix::StereoProjectionMatrix( const cv::Mat &leftProjectionMatrix, const cv::Mat &rightProjectionMatrix )
    : m_leftProjectionMatrix( leftProjectionMatrix ), m_rightProjectionMatrix( rightProjectionMatrix )
{
}

StereoProjectionMatrix::StereoProjectionMatrix( const std::string &fileName )
{
    loadYaml( fileName );
}

void StereoProjectionMatrix::setLeftProjectionMatrix( const cv::Mat &value )
{
    m_leftProjectionMatrix.setProjectionMatrix( value );
}

void StereoProjectionMatrix::setLeftProjectionMatrix( const ProjectionMatrix &value )
{
    m_leftProjectionMatrix = value;
}

void StereoProjectionMatrix::setRightProjectionMatrix( const cv::Mat &value )
{
    m_rightProjectionMatrix.setProjectionMatrix( value );
}

void StereoProjectionMatrix::setRightProjectionMatrix( const ProjectionMatrix &value )
{
    m_rightProjectionMatrix = value;
}

void StereoProjectionMatrix::multiplicateCameraMatrix( const double value )
{
    m_leftProjectionMatrix.multiplicateCameraMatrix( value );
    m_rightProjectionMatrix.multiplicateCameraMatrix( value );
}

void StereoProjectionMatrix::movePrincipalPoint( const cv::Vec2f &value )
{
    m_leftProjectionMatrix.movePrincipalPoint( value );
    m_rightProjectionMatrix.movePrincipalPoint( value );
}

void StereoProjectionMatrix::rotate( const cv::Mat &mat )
{
    m_leftProjectionMatrix.rotate( mat );
    m_rightProjectionMatrix.rotate( mat );
}

void StereoProjectionMatrix::translate( const cv::Mat &vec )
{
    m_leftProjectionMatrix.translate( vec );
    m_rightProjectionMatrix.translate( vec );
}

const ProjectionMatrix &StereoProjectionMatrix::leftProjectionMatrix() const
{
    return m_leftProjectionMatrix;
}

const ProjectionMatrix &StereoProjectionMatrix::rightProjectionMatrix() const
{
    return m_rightProjectionMatrix;
}

cv::Mat StereoProjectionMatrix::baselineVector() const
{
    return m_rightProjectionMatrix.translation() - m_leftProjectionMatrix.translation();
}

double StereoProjectionMatrix::baselineVectorLenght() const
{
    return cv::norm( baselineVector() );
}

cv::Mat StereoProjectionMatrix::disparityToDepthMatrix() const
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

bool StereoProjectionMatrix::saveYaml( const std::string &fileName ) const
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

bool StereoProjectionMatrix::loadYaml( const std::string &fileName )
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

bool StereoProjectionMatrix::operator==( const StereoProjectionMatrix &other ) const
{
    return m_leftProjectionMatrix == other.m_leftProjectionMatrix && m_rightProjectionMatrix == other.m_rightProjectionMatrix ;
}

bool StereoProjectionMatrix::operator!=( const StereoProjectionMatrix &other ) const
{
    return !operator==( other ) ;
}

std::ostream &operator<<( std::ostream& out, const StereoProjectionMatrix& matrix )
{
    out << matrix.m_leftProjectionMatrix << matrix.m_rightProjectionMatrix;

    return out;
}
