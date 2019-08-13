/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include "src/common/fileslistwidget.h"
#include <QApplication>
#include <QMessageBox>
#include <QFile>
#include <QDebug>

#include "src/ORB/System.h"

#include "src/common/calibrationdatabase.h"

using namespace std;

std::string pathToVocabulary = "/home/victor/calibration/ORBvoc.txt";
std::string pathToSettings = "/home/victor/calibration/calibration.yaml";
QString pathToImages = "/home/victor/datasets/Polygon/";

int main( int argc, char **argv )
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

        StereoCalibrationDataShort calibrationData( pathToSettings );

        if ( calibrationData.isOk() ) {

            ORB_SLAM2::Settings settings( calibrationData );

            settings.setCx( settings.cx() - calibrationData.leftROI().x );
            settings.setCy( settings.cy() - calibrationData.leftROI().y );

            settings.zeroDistortionCoefficients();

            ORB_SLAM2::System SLAM( pathToVocabulary, settings, ORB_SLAM2::System::STEREO, true );

            cv::Mat imLeft, imRight;

            double tframe = 0;

            for( int i = 10000; i < 30000; ++i )
            {
                auto vstrImageLeft = leftDir + "/" + QString::number(i) + "_left.jpg";
                auto vstrImageRight = rightDir + "/" + QString::number(i) + "_right.jpg";

                qDebug() << vstrImageLeft << vstrImageRight;

                imLeft = cv::imread( vstrImageLeft.toStdString(), cv::IMREAD_UNCHANGED );
                imRight = cv::imread( vstrImageRight.toStdString(), cv::IMREAD_UNCHANGED );

                if ( imLeft.empty() ||  imRight.empty() ) {
                    std::cerr << "Failed to load image: " << i << std::endl;
                    return 1;
                }

                CvImage leftRectifiedImage;
                CvImage rightRectifiedImage;

                cv::remap( imLeft, leftRectifiedImage, calibrationData.leftRMap(), calibrationData.leftDMap(), cv::INTER_LANCZOS4 );
                cv::remap( imRight, rightRectifiedImage, calibrationData.rightRMap(), calibrationData.rightDMap(), cv::INTER_LANCZOS4 );

                CvImage leftCroppedFrame;
                CvImage rightCroppedFrame;

                if (!calibrationData.leftROI().empty() && !calibrationData.rightROI().empty()) {
                    leftCroppedFrame = leftRectifiedImage( calibrationData.leftROI() );
                    rightCroppedFrame = rightRectifiedImage( calibrationData.leftROI() );
                }

                tframe += 1.0/10.0;

                SLAM.TrackStereo( leftCroppedFrame, rightCroppedFrame, tframe );

            }


            getchar();

            SLAM.Shutdown();

        }

    }

    return 0;
}

