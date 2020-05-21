#include "src/common/precompiled.h"

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

unsigned int MonocularCalibrationData::resultsSize() const
{
    return m_results.size();
}

void MonocularCalibrationData::setPreviewImage( const CvImage &image )
{
    m_previewImage = image;
}

const CvImage &MonocularCalibrationData::previewImage() const
{
    return m_previewImage;
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

void StereoCalibrationData::setLeftCameraResults(const MonocularCalibrationData &value )
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

        setLeftROI( leftROI );
        setRightROI( rightROI );

        setError( error );

        setOk( true );

        return true;
    }

    return false;

}

StereoCalibrationData::operator StereoCalibrationDataShort() const
{
    StereoCalibrationDataShort ret( *this );

    ret.setLeftCameraResults( m_leftCameraResults );
    ret.setRightCameraResults( m_rightCameraResults );

    return ret;

}



