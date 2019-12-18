#include "precompiled.h"

#include "projectionmatrix.h"

#include <opencv2/sfm.hpp>

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

void ProjectionMatrix::movePrincipalPoint( const cv::Vec2f &value )
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

Plane ProjectionMatrix::plane() const
{
    auto a = m_r.at< double >( 0, 2 );
    auto b = m_r.at< double >( 1, 2 );
    auto c = m_r.at< double >( 2, 2 );

    return Plane( a, b, c, -a * m_t.at< double >( 0, 0 ) - b * m_t.at< double >( 1, 0 ) - c * m_t.at< double >( 2, 0 ) );

}

ProjectionMatrix::operator cv::Mat() const
{
    return projectionMatrix();
}
