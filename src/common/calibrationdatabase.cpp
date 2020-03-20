#include "precompiled.h"

#include "calibrationdatabase.h"

// MonocularCalibrationDataShort
CalibrationDataBase::CalibrationDataBase()
{
    initialize();
}

void CalibrationDataBase::initialize()
{
    m_ok = false;
}

void CalibrationDataBase::setOk( const bool value )
{
    m_ok = value;
}

bool CalibrationDataBase::isOk() const
{
    return m_ok;
}

void CalibrationDataBase::setError( const double value )
{
    m_error = value;
}

double CalibrationDataBase::error() const
{
    return m_error;
}

// MonocularCalibrationDataShort
MonocularCalibrationDataShort::MonocularCalibrationDataShort()
{
    initialize();
}

MonocularCalibrationDataShort::MonocularCalibrationDataShort( const std::string &fileName )
{
    initialize();

    loadYaml( fileName );
}

void MonocularCalibrationDataShort::initialize()
{
}

void MonocularCalibrationDataShort::setFrameSize( const cv::Size &value )
{
    m_frameSize = value;
}

const cv::Size &MonocularCalibrationDataShort::frameSize() const
{
    return m_frameSize;
}

void MonocularCalibrationDataShort::setCameraMatrix( const cv::Mat &value )
{
    m_cameraMatrix = value;
}

const cv::Mat &MonocularCalibrationDataShort::cameraMatrix() const
{
    return m_cameraMatrix;
}

void MonocularCalibrationDataShort::setDistortionCoefficients( const cv::Mat &value )
{
    m_distCoefficients = value;
}

const cv::Mat &MonocularCalibrationDataShort::distortionCoefficients() const
{
    return m_distCoefficients;
}

int MonocularCalibrationDataShort::frameWidth() const
{
    return m_frameSize.width;
}

int MonocularCalibrationDataShort::frameHeight() const
{
    return m_frameSize.height;
}

void MonocularCalibrationDataShort::setFx( const double value )
{
    m_cameraMatrix.at< double >( 0, 0 ) = value;
}

void MonocularCalibrationDataShort::setFy( const double value )
{
    m_cameraMatrix.at< double >( 1, 1 ) = value;
}

void MonocularCalibrationDataShort::setCx( const double value )
{
    m_cameraMatrix.at< double >( 0, 2 ) = value;
}

void MonocularCalibrationDataShort::setCy( const double value )
{
    m_cameraMatrix.at< double >( 1, 2 ) = value;
}

double MonocularCalibrationDataShort::fx() const
{
    return m_cameraMatrix.at< double >( 0, 0 );
}

double MonocularCalibrationDataShort::fy() const
{
    return m_cameraMatrix.at< double >( 1, 1 );
}

double MonocularCalibrationDataShort::cx() const
{
    return m_cameraMatrix.at< double >( 0, 2 );
}

double MonocularCalibrationDataShort::cy() const
{
    return m_cameraMatrix.at< double >( 1, 2 );
}

void MonocularCalibrationDataShort::shiftPrincipalValues( const cv::Point2i &value )
{
    m_cameraMatrix.at< double >( 0, 2 ) += value.x;
    m_cameraMatrix.at< double >( 1, 2 ) += value.y;
}

double MonocularCalibrationDataShort::k1() const
{
    return m_distCoefficients.at< double >( 0 );
}

double MonocularCalibrationDataShort::k2() const
{
    return m_distCoefficients.at< double >( 1 );
}

double MonocularCalibrationDataShort::k3() const
{
    if ( m_distCoefficients.elemSize() > 4 )
        return m_distCoefficients.at<double>( 4 );
    else
        return 0.0;
}

double MonocularCalibrationDataShort::k4() const
{
    if ( m_distCoefficients.elemSize() > 5 )
        return m_distCoefficients.at<double>( 5 );
    else
        return 0.0;
}

double MonocularCalibrationDataShort::k5() const
{
    if ( m_distCoefficients.elemSize() > 6 )
        return m_distCoefficients.at<double>( 6 );
    else
        return 0.0;
}

double MonocularCalibrationDataShort::k6() const
{
    if ( m_distCoefficients.elemSize() > 7 )
        return m_distCoefficients.at<double>( 7 );
    else
        return 0.0;
}

double MonocularCalibrationDataShort::p1() const
{
    return m_distCoefficients.at<double>( 2 );
}

double MonocularCalibrationDataShort::p2() const
{
    return m_distCoefficients.at<double>( 3 );
}

bool MonocularCalibrationDataShort::saveYaml( const std::string &fileName ) const
{
    if ( isOk() ) {
        cv::FileStorage fs( fileName, cv::FileStorage::WRITE );

        time_t rawtime; time( &rawtime );
        fs << "calibrationDate" << asctime( localtime( &rawtime ) );

        fs << "frameSize" << frameSize();

        fs << "cameraMatrix" << cameraMatrix();
        fs << "distortionCoefficients" << distortionCoefficients();

        fs << "error" << error();

        fs.release();

        return true;
    }

    return false;

}

bool MonocularCalibrationDataShort::loadYaml( const std::string &fileName )
{
    cv::FileStorage fs ( fileName, cv::FileStorage::READ );

    if ( fs.isOpened() ) {

        cv::Size frameSize;
        fs["frameSize"] >> frameSize;
        setFrameSize( frameSize );

        cv::Mat cameraMatrix, distortionCoefficients;
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distortionCoefficients"] >> distortionCoefficients;

        setCameraMatrix( cameraMatrix );
        setDistortionCoefficients( distortionCoefficients );

        double error;

        fs["error"] >> error;

        setError( error );

        setOk( true );

        return true;
    }

    return false;

}

// StereoCalibrationDataBase
StereoCalibrationDataBase::StereoCalibrationDataBase()
{
    initialize();
}

void StereoCalibrationDataBase::initialize()
{
}

void StereoCalibrationDataBase::setCorrespondFrameCount( const unsigned int value )
{
    m_correspondFrameCount = value;
}

unsigned int StereoCalibrationDataBase::correspondFrameCount() const
{
    return m_correspondFrameCount;
}

void StereoCalibrationDataBase::setRotationMatrix( const cv::Mat &value )
{
    m_rotationMatrix = value;
}

const cv::Mat &StereoCalibrationDataBase::rotationMatrix() const
{
    return m_rotationMatrix;
}

void StereoCalibrationDataBase::setTranslationVector( const cv::Mat &value )
{
    m_translationVector = value;
}

const cv::Mat &StereoCalibrationDataBase::translationVector() const
{
    return m_translationVector;
}

const cv::Mat StereoCalibrationDataBase::baselineVector() const
{
    return cv::Mat_< double >( 3, 1 ) << ( m_translationVector.at< double >( 0 ) > 0 ? distance() : -distance() ), 0.0, 0.0;
}

void StereoCalibrationDataBase::setFundamentalMatrix( const cv::Mat &value )
{
    m_fundamentalMatrix = value;
}

const cv::Mat &StereoCalibrationDataBase::fundamentalMatrix() const
{
    return m_fundamentalMatrix;
}

void StereoCalibrationDataBase::setEssentialMatrix( const cv::Mat &value )
{
    m_essentialMatrix = value;
}

const cv::Mat &StereoCalibrationDataBase::essentialMatrix() const
{
    return m_essentialMatrix;
}

void StereoCalibrationDataBase::setLeftRectifyMatrix( const cv::Mat &value )
{
    m_leftRectifyMatrix = value;
}

const cv::Mat &StereoCalibrationDataBase::leftRectifyMatrix() const
{
    return m_leftRectifyMatrix;
}

void StereoCalibrationDataBase::setRightRectifyMatrix( const cv::Mat &value )
{
    m_rightRectifyMatrix = value;
}

const cv::Mat &StereoCalibrationDataBase::rightRectifyMatrix() const
{
    return m_rightRectifyMatrix;
}

void StereoCalibrationDataBase::setLeftProjectionMatrix( const cv::Mat &value )
{
    m_projectionMatrix.setLeftProjectionMatrix( value );
}

void StereoCalibrationDataBase::setLeftProjectionMatrix( const ProjectionMatrix &value )
{
    m_projectionMatrix.setLeftProjectionMatrix( value );
}

const ProjectionMatrix &StereoCalibrationDataBase::leftProjectionMatrix() const
{
    return m_projectionMatrix.leftProjectionMatrix();
}

void StereoCalibrationDataBase::setRightProjectionMatrix( const cv::Mat &value )
{
    m_projectionMatrix.setRightProjectionMatrix( value );
}

void StereoCalibrationDataBase::setRightProjectionMatrix( const ProjectionMatrix &value )
{
    m_projectionMatrix.setRightProjectionMatrix( value );
}

const ProjectionMatrix &StereoCalibrationDataBase::rightProjectionMatrix() const
{
    return m_projectionMatrix.rightProjectionMatrix();
}

const StereoCameraMatrix &StereoCalibrationDataBase::projectionMatrix() const
{
    return m_projectionMatrix;
}

void StereoCalibrationDataBase::setDisparityToDepthMatrix( const cv::Mat &value )
{
    m_disparityToDepthMatrix = value;
}

const cv::Mat &StereoCalibrationDataBase::disparityToDepthMatrix() const
{
    return m_disparityToDepthMatrix;
}

void StereoCalibrationDataBase::setLeftROI( const cv::Rect &value )
{
    m_leftROI = value;
}

const cv::Rect &StereoCalibrationDataBase::leftROI() const
{
    return m_leftROI;
}

void StereoCalibrationDataBase::setRightROI( const cv::Rect &value )
{
    m_rightROI = value;
}

const cv::Rect &StereoCalibrationDataBase::rightROI() const
{
    return m_rightROI;
}

double StereoCalibrationDataBase::distance() const
{
    auto xTrans = m_translationVector.at<double>( 0 );
    auto yTrans = m_translationVector.at<double>( 1 );
    auto zTrans = m_translationVector.at<double>( 2 );

    return sqrt( xTrans * xTrans + yTrans * yTrans + zTrans * zTrans );
}

cv::Rect StereoCalibrationDataBase::cropRect() const
{
    if ( leftROI().empty() || rightROI().empty() )
        return cv::Rect();

    cv::Rect ret;

    ret.x = std::max( m_leftROI.x, m_rightROI.x );
    ret.y = std::max( m_leftROI.y, m_rightROI.y );
    ret.width = std::min( m_leftROI.x + m_leftROI.width, m_rightROI.x + m_rightROI.width ) - ret.x;
    ret.height = std::min( m_leftROI.y + m_leftROI.height, m_rightROI.y + m_rightROI.height ) - ret.y;

    return ret;
}

// StereoCalibrationDataShort
StereoCalibrationDataShort::StereoCalibrationDataShort()
{
    initialize();
}

StereoCalibrationDataShort::StereoCalibrationDataShort( const std::string &fileName )
{
    initialize();

    loadYaml( fileName );
}

void StereoCalibrationDataShort::initialize()
{

}

void StereoCalibrationDataShort::setLeftCameraResults(const MonocularCalibrationDataShort &value )
{
    m_leftCameraResults = value;
}

const MonocularCalibrationDataShort &StereoCalibrationDataShort::leftCameraResults() const
{
    return m_leftCameraResults;
}

MonocularCalibrationDataShort &StereoCalibrationDataShort::leftCameraResults()
{
    return m_leftCameraResults;
}

void StereoCalibrationDataShort::setRightCameraResults( const MonocularCalibrationDataShort &value )
{
    m_rightCameraResults = value;
}

const MonocularCalibrationDataShort &StereoCalibrationDataShort::rightCameraResults() const
{
    return m_rightCameraResults;
}

MonocularCalibrationDataShort &StereoCalibrationDataShort::rightCameraResults()
{
    return m_rightCameraResults;
}

bool StereoCalibrationDataShort::saveYaml( const std::string &fileName ) const
{
    if ( isOk() ) {
        cv::FileStorage fs( fileName, cv::FileStorage::WRITE );

        time_t rawtime; time( &rawtime );
        fs << "calibrationDate" << asctime( localtime( &rawtime ) );

        fs << "correspondFrameCount" << static_cast<int>( correspondFrameCount() );

        fs << "frameSize" << leftCameraResults().frameSize();

        fs << "leftCameraMatrix" << leftCameraResults().cameraMatrix();
        fs << "leftDistortionCoefficients" << leftCameraResults().distortionCoefficients();
        fs << "leftError" << leftCameraResults().error();

        fs << "rightCameraMatrix" << rightCameraResults().cameraMatrix();
        fs << "rightDistortionCoefficients" << rightCameraResults().distortionCoefficients();
        fs << "rightError" << rightCameraResults().error();

        fs << "rotationMatrix" << rotationMatrix();
        fs << "translationVector" << translationVector();
        fs << "fundamentalMatrix" << fundamentalMatrix();
        fs << "essentialMatrix" << essentialMatrix();
        fs << "leftRectifyMatrix" << leftRectifyMatrix();
        fs << "rightRectifyMatrix" << rightRectifyMatrix();
        fs << "leftProjectionMatrix" << leftProjectionMatrix();
        fs << "rightProjectionMatrix" << rightProjectionMatrix();
        fs << "disparityToDepthMatrix" << disparityToDepthMatrix();
        fs << "leftROI" << leftROI();
        fs << "rightROI" << rightROI();

        fs << "error" << error();

        fs.release();

        return true;

    }

    return false;

}

bool StereoCalibrationDataShort::loadYaml( const std::string &fileName )
{
    cv::FileStorage fs ( fileName, cv::FileStorage::READ );

    if ( fs.isOpened() ) {

        int correspondFrameCount;
        fs["correspondFrameCount"] >> correspondFrameCount;

        cv::Size frameSize;
        fs["frameSize"] >> frameSize;

        leftCameraResults().setFrameSize( frameSize );
        rightCameraResults().setFrameSize( frameSize );

        cv::Mat leftCameraMatrix, leftDistortionCoefficients;
        double leftError;
        fs["leftCameraMatrix"] >> leftCameraMatrix;
        fs["leftDistortionCoefficients"] >> leftDistortionCoefficients;
        fs["leftError"] >> leftError;

        leftCameraResults().setCameraMatrix( leftCameraMatrix );
        leftCameraResults().setDistortionCoefficients( leftDistortionCoefficients );
        leftCameraResults().setError( leftError );
        leftCameraResults().setOk( true );

        cv::Mat rightCameraMatrix, rightDistortionCoefficients;
        double rightError;
        fs["rightCameraMatrix"] >> rightCameraMatrix;
        fs["rightDistortionCoefficients"] >> rightDistortionCoefficients;
        fs["rightError"] >> rightError;

        rightCameraResults().setCameraMatrix( rightCameraMatrix );
        rightCameraResults().setDistortionCoefficients( rightDistortionCoefficients );
        rightCameraResults().setError( rightError );
        rightCameraResults().setOk( true );

        cv::Mat rotationMatrix;
        cv::Mat translationVector;
        cv::Mat fundamentalMatrix;
        cv::Mat essentialMatrix;

        cv::Mat leftRectifyMatrix;
        cv::Mat rightRectifyMatrix;
        cv::Mat leftProjectionMatrix;
        cv::Mat rightProjectionMatrix;
        cv::Mat disparityToDepthMatrix;

        cv::Rect leftROI;
        cv::Rect rightROI;

        double error;

        fs["rotationMatrix"] >> rotationMatrix;
        fs["translationVector"] >> translationVector;
        fs["fundamentalMatrix"] >> fundamentalMatrix;
        fs["essentialMatrix"] >> essentialMatrix;
        fs["leftRectifyMatrix"] >> leftRectifyMatrix;
        fs["rightRectifyMatrix"] >> rightRectifyMatrix;
        fs["leftProjectionMatrix"] >> leftProjectionMatrix;
        fs["rightProjectionMatrix"] >> rightProjectionMatrix;
        fs["disparityToDepthMatrix"] >> disparityToDepthMatrix;
        fs["leftROI"] >> leftROI;
        fs["rightROI"] >> rightROI;

        fs["error"] >> error;

        setRotationMatrix( rotationMatrix );
        setTranslationVector( translationVector );
        setFundamentalMatrix( fundamentalMatrix );
        setEssentialMatrix( essentialMatrix );

        setLeftRectifyMatrix( leftRectifyMatrix );
        setRightRectifyMatrix( rightRectifyMatrix );
        setLeftProjectionMatrix( leftProjectionMatrix );
        setRightProjectionMatrix( rightProjectionMatrix );
        setDisparityToDepthMatrix( disparityToDepthMatrix );

        setLeftROI( leftROI );
        setRightROI( rightROI );

        setError( error );

        setOk( true );

        return true;
    }

    return false;

}
