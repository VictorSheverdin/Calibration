#include "precompiled.h"

#include "calibrationdata.h"

// MonocularCalibrationData
MonocularCalibrationResult::MonocularCalibrationResult()
{
    initialize();
}

void MonocularCalibrationResult::initialize()
{
    m_ok = false;
}

void MonocularCalibrationResult::setRVec( const cv::Mat &value )
{
    m_rVec = value;
}

const cv::Mat &MonocularCalibrationResult::rVec() const
{
    return m_rVec;
}

void MonocularCalibrationResult::setTVec( const cv::Mat &value )
{
    m_tVec = value;
}

const cv::Mat &MonocularCalibrationResult::tVec() const
{
    return m_tVec;
}

void MonocularCalibrationResult::setPoints2d( const std::vector< cv::Point2f > &value )
{
    m_points2d = value;
}

const std::vector< cv::Point2f > &MonocularCalibrationResult::points2d() const
{
    return m_points2d;
}

void MonocularCalibrationResult::setPoints3d( const std::vector< cv::Point3f > &value )
{
    m_points3d = value;
}

const std::vector< cv::Point3f > &MonocularCalibrationResult::points3d() const
{
    return m_points3d;
}

void MonocularCalibrationResult::setOk( const bool value )
{
    m_ok = value;
}

bool MonocularCalibrationResult::isOk() const
{
    return m_ok;
}

// MonocularCalibrationData
MonocularCalibrationData::MonocularCalibrationData()
{
    initialize();
}

void MonocularCalibrationData::initialize()
{
    m_ok = false;
}

void MonocularCalibrationData::setFrameSize( const cv::Size &value )
{
    m_frameSize = value;
}

const cv::Size &MonocularCalibrationData::frameSize() const
{
    return m_frameSize;
}

void MonocularCalibrationData::setCameraMatrix( const cv::Mat &value )
{
    m_cameraMatrix = value;
}

const cv::Mat &MonocularCalibrationData::cameraMatrix() const
{
    return m_cameraMatrix;
}

void MonocularCalibrationData::setDistortionCoefficients( const cv::Mat &value )
{
    m_distCoefficients = value;
}

const cv::Mat &MonocularCalibrationData::distortionCoefficients() const
{
    return m_distCoefficients;
}

void MonocularCalibrationData::setResults( std::vector< MonocularCalibrationResult > &value )
{
    m_results = value;
}

const std::vector< MonocularCalibrationResult > &MonocularCalibrationData::results() const
{
    return m_results;
}

MonocularCalibrationResult &MonocularCalibrationData::result( const unsigned int i )
{
    return m_results[i];
}

const MonocularCalibrationResult &MonocularCalibrationData::result( const unsigned int i ) const
{
    return m_results[i];
}

void MonocularCalibrationData::setOk( const bool value )
{
    m_ok = value;
}

bool MonocularCalibrationData::isOk() const
{
    return m_ok;
}

void MonocularCalibrationData::setError( const double value )
{
    m_error = value;
}

double MonocularCalibrationData::error() const
{
    return m_error;
}

const unsigned int MonocularCalibrationData::resultsSize() const
{
    return m_results.size();
}


bool MonocularCalibrationData::saveYaml( const std::string &fileName ) const
{
    if ( isOk() ) {
        cv::FileStorage fs( fileName, cv::FileStorage::WRITE );

        time_t rawtime; time( &rawtime );
        fs << "calibrationDate" << asctime( localtime( &rawtime ) );

        fs << "frameCount" << static_cast<int>( resultsSize() );

        fs << "frameSize" << frameSize();

        fs << "cameraMatrix" << cameraMatrix();
        fs << "distortionCoefficients" << distortionCoefficients();

        fs << "error" << error();

        fs.release();

        return true;
    }

    return false;

}

bool MonocularCalibrationData::loadYaml( const std::string &fileName )
{
    cv::FileStorage fs ( fileName, cv::FileStorage::READ );

    if ( fs.isOpened() ) {

        int frameCount;
        fs["frameCount"] >> frameCount;

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

// StereoCalibrationData
StereoCalibrationData::StereoCalibrationData()
{
}

void StereoCalibrationData::setLeftCameraResults( const MonocularCalibrationData &value )
{
    m_leftCameraResults = value;
}

const MonocularCalibrationData &StereoCalibrationData::leftCameraResults() const
{
    return m_leftCameraResults;
}

MonocularCalibrationData &StereoCalibrationData::leftCameraResults()
{
    return m_leftCameraResults;
}

void StereoCalibrationData::setRightCameraResults( const MonocularCalibrationData &value )
{
    m_rightCameraResults = value;
}

const MonocularCalibrationData &StereoCalibrationData::rightCameraResults() const
{
    return m_rightCameraResults;
}

MonocularCalibrationData &StereoCalibrationData::rightCameraResults()
{
    return m_rightCameraResults;
}

unsigned int StereoCalibrationData::leftResultsSize() const
{
    return m_leftCameraResults.resultsSize();
}

unsigned int StereoCalibrationData::rightResultsSize() const
{
    return m_rightCameraResults.resultsSize();
}

void StereoCalibrationData::setCorrespondFrameCount( const unsigned int value )
{
    m_correspondFrameCount = value;
}

unsigned int StereoCalibrationData::correspondFrameCount() const
{
    return m_correspondFrameCount;
}

void StereoCalibrationData::setRotationMatrix( const cv::Mat &value )
{
    m_rotationMatrix = value;
}

const cv::Mat &StereoCalibrationData::rotationMatrix() const
{
    return m_rotationMatrix;
}

void StereoCalibrationData::setTranslationVector( const cv::Mat &value )
{
    m_translationVector = value;
}

const cv::Mat &StereoCalibrationData::translationVector() const
{
    return m_translationVector;
}

void StereoCalibrationData::setFundamentalMatrix( const cv::Mat &value )
{
    m_fundamentalMatrix = value;
}

const cv::Mat &StereoCalibrationData::fundamentalMatrix() const
{
    return m_fundamentalMatrix;
}

void StereoCalibrationData::setEssentialMatrix( const cv::Mat &value )
{
    m_essentialMatrix = value;
}

const cv::Mat &StereoCalibrationData::essentialMatrix() const
{
    return m_essentialMatrix;
}

void StereoCalibrationData::setLeftRectifyMatrix( const cv::Mat &value )
{
    m_leftRectifyMatrix = value;
}

const cv::Mat &StereoCalibrationData::leftRectifyMatrix() const
{
    return m_leftRectifyMatrix;
}

void StereoCalibrationData::setRightRectifyMatrix( const cv::Mat &value )
{
    m_rightRectifyMatrix = value;
}

const cv::Mat &StereoCalibrationData::rightRectifyMatrix() const
{
    return m_rightRectifyMatrix;
}

void StereoCalibrationData::setLeftProjectionMatrix( const cv::Mat &value )
{
    m_leftProjectionMatrix = value;
}

const cv::Mat &StereoCalibrationData::leftProjectionMatrix() const
{
    return m_leftProjectionMatrix;
}

void StereoCalibrationData::setRightProjectionMatrix( const cv::Mat &value )
{
    m_rightProjectionMatrix = value;
}

const cv::Mat &StereoCalibrationData::rightProjectionMatrix() const
{
    return m_rightProjectionMatrix;
}

void StereoCalibrationData::setDisparityToDepthMatrix( const cv::Mat &value )
{
    m_disparityToDepthMatrix = value;
}

const cv::Mat &StereoCalibrationData::disparityToDepthMatrix() const
{
    return m_disparityToDepthMatrix;
}

void StereoCalibrationData::setLeftROI( const cv::Rect &value )
{
    m_leftROI = value;
}

const cv::Rect &StereoCalibrationData::leftROI() const
{
    return m_leftROI;
}

void StereoCalibrationData::setRightROI( const cv::Rect &value )
{
    m_rightROI = value;
}

const cv::Rect &StereoCalibrationData::rightROI() const
{
    return m_rightROI;
}

void StereoCalibrationData::setLeftRMap( const cv::Mat &value )
{
    m_leftRMap = value;
}

const cv::Mat &StereoCalibrationData::leftRMap() const
{
    return m_leftRMap;
}

void StereoCalibrationData::setLeftDMap( const cv::Mat &value )
{
    m_leftDMap = value;
}

const cv::Mat &StereoCalibrationData::leftDMap() const
{
    return m_leftDMap;
}

void StereoCalibrationData::setRightRMap( const cv::Mat &value )
{
    m_rightRMap = value;
}

const cv::Mat &StereoCalibrationData::rightRMap() const
{
    return m_rightRMap;
}

void StereoCalibrationData::setRightDMap( const cv::Mat &value )
{
    m_rightDMap = value;
}

const cv::Mat &StereoCalibrationData::rightDMap() const
{
    return m_rightDMap;
}

void StereoCalibrationData::setError( const double value )
{
    m_error = value;
}

double StereoCalibrationData::error() const
{
    return m_error;
}

bool StereoCalibrationData::isOk() const
{
    return m_leftCameraResults.isOk() && m_rightCameraResults.isOk();
}

bool StereoCalibrationData::saveYaml( const std::string &fileName ) const
{
    if ( isOk() ) {
        cv::FileStorage fs( fileName, cv::FileStorage::WRITE );

        time_t rawtime; time( &rawtime );
        fs << "calibrationDate" << asctime( localtime( &rawtime ) );

        fs << "correspondFrameCount" << static_cast<int>( correspondFrameCount() );

        fs << "frameSize" << leftCameraResults().frameSize();

        fs << "leftFrameCount" << static_cast<int>( leftCameraResults().resultsSize() );
        fs << "leftCameraMatrix" << leftCameraResults().cameraMatrix();
        fs << "leftDistortionCoefficients" << leftCameraResults().distortionCoefficients();
        fs << "leftError" << leftCameraResults().error();

        fs << "rightFrameCount" << static_cast<int>( rightCameraResults().resultsSize() );
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

bool StereoCalibrationData::loadYaml( const std::string &fileName )
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

        cv::Mat leftRMap, leftDMap;

        cv::initUndistortRectifyMap( leftCameraMatrix, leftDistortionCoefficients,
                                     leftRectifyMatrix, leftProjectionMatrix, frameSize,
                                     CV_32FC2, leftRMap, leftDMap );

        setLeftRMap( leftRMap );
        setLeftDMap( leftDMap );

        cv::Mat rightRMap, rightDMap;

        cv::initUndistortRectifyMap( rightCameraMatrix, rightDistortionCoefficients,
                                     rightRectifyMatrix, rightProjectionMatrix, frameSize,
                                     CV_32FC2, rightRMap, rightDMap );

        setRightRMap( rightRMap );
        setRightDMap( rightDMap );

        setError( error );

        return true;
    }

    return false;

}


