#include "src/common/precompiled.h"

#include <memory>

#include <QApplication>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Rtabmap.h>

#include "src/common/calibrationdatabase.h"
#include "src/common/fileslistwidget.h"

std::string pathToSettings = "/home/victor/calibration/calibration.yaml";

int main(int argc, char** argv)
{
    QString leftDir;
    QString rightDir;

    {
        QApplication a( argc, argv );

        QFile cssFile(":/resources/qss/style.css");

        if ( cssFile.open( QIODevice::ReadOnly ) ) {
            QString cssString( cssFile.readAll() );
            a.setStyleSheet( cssString );
        }
        else
            QMessageBox::critical( nullptr, "Error", "Can't load css file:" + cssFile.fileName() );

        StereoDirDialog dlg;

        if ( dlg.exec() == QDialog::Accepted ) {
            leftDir = dlg.leftDir();
            rightDir = dlg.rightDir();
        }

    }

    if ( !leftDir.isEmpty() && !rightDir.isEmpty() ) {
        StereoCalibrationDataShort calibrationParameters( pathToSettings );

        if ( calibrationParameters.isOk() ) {

            rtabmap::ParametersMap parameters;
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kRtabmapDetectionRate(), "5" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kRtabmapImagesAlreadyRectified(), "true" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kKpDetectorStrategy(), "2" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kORBNLevels(), "24" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kOdomStrategy(), "0" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kOdomResetCountdown(), "1" ) );
            /*parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoOpticalFlow(), "false" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoDenseStrategy(), "0" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMNumDisparities(), "256" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMBlockSize(), "11" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMPreFilterSize(), "15" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMPreFilterCap(), "63" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMTextureThreshold(), "0" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMUniquenessRatio(), "27" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMSpeckleWindowSize(), "45" ) );
            parameters.insert( rtabmap::ParametersPair( rtabmap::Parameters::kStereoBMSpeckleRange(), "10" ) );*/

            cv::Point2i principalShift( -calibrationParameters.leftROI().x, -calibrationParameters.leftROI().y  );

            calibrationParameters.leftCameraResults().shiftPrincipalValues( principalShift );
            calibrationParameters.rightCameraResults().shiftPrincipalValues( principalShift );

            rtabmap::CameraModel leftCamera( "LeftCamera", calibrationParameters.leftCameraResults().frameSize(),
                                             calibrationParameters.leftCameraResults().cameraMatrix(),
                                             calibrationParameters.leftCameraResults().distortionCoefficients().t(),
                                             calibrationParameters.leftRectifyMatrix(),
                                             calibrationParameters.leftProjectionMatrix() );

            rtabmap::CameraModel rightCamera( "RightCamera", calibrationParameters.rightCameraResults().frameSize(),
                                              calibrationParameters.rightCameraResults().cameraMatrix(),
                                              calibrationParameters.rightCameraResults().distortionCoefficients().t(),
                                              calibrationParameters.rightRectifyMatrix(),
                                              calibrationParameters.rightProjectionMatrix() );

            rtabmap::StereoCameraModel stereoCamera( "StereoCamera", leftCamera, rightCamera,
                                                     calibrationParameters.rotationMatrix(), calibrationParameters.translationVector(),
                                                     calibrationParameters.essentialMatrix(), calibrationParameters.fundamentalMatrix() );

            std::unique_ptr< rtabmap::Odometry > odometry( rtabmap::Odometry::create( parameters ) );

            rtabmap::Rtabmap rtabmap;
            rtabmap.init( parameters, "/home/victor/rtabtest/test6.db" );

            int id = 0;
            double t = 0;

            cv::Mat imLeft, imRight;

            auto start = 10800;
            for( int i = start; i < start + 1000; ++i ) {
                auto vstrImageLeft = leftDir + "/" + QString::number( i ) + "_left.jpg";
                auto vstrImageRight = rightDir + "/" + QString::number( i ) + "_right.jpg";

                imLeft = cv::imread( vstrImageLeft.toStdString(), cv::IMREAD_COLOR );
                imRight = cv::imread( vstrImageRight.toStdString(), cv::IMREAD_GRAYSCALE );

                if ( imLeft.empty() || imRight.empty() ) {
                    std::cerr << "Failed to load image: " << i << std::endl;
                    return 1;
                }

                CvImage leftRectifiedImage;
                CvImage rightRectifiedImage;

                cv::remap( imLeft, leftRectifiedImage, calibrationParameters.leftRMap(), calibrationParameters.leftDMap(), cv::INTER_LANCZOS4 );
                cv::remap( imRight, rightRectifiedImage, calibrationParameters.rightRMap(), calibrationParameters.rightDMap(), cv::INTER_LANCZOS4 );

                CvImage leftCroppedFrame;
                CvImage rightCroppedFrame;

                leftCroppedFrame = leftRectifiedImage( calibrationParameters.leftROI() );
                rightCroppedFrame = rightRectifiedImage( calibrationParameters.leftROI() );

                rtabmap::SensorData sensorData( leftCroppedFrame, rightCroppedFrame, stereoCamera, id, t );

                rtabmap::OdometryInfo odometryInfo;
                rtabmap::Transform pose = odometry->process( sensorData, &odometryInfo );

                std::cout << i << std::endl;
                std::cout << pose << std::endl;

                if ( i % 10 == 0 )
                    rtabmap.process( sensorData, pose );

                ++id;
                t += 1.0/10.0;

            }

        }

    }

    return 0;
}

