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

#include "src/ORB/System.h"

using namespace std;

std::string pathToVocabulary = "/home/victor/calibration/ORBvoc.txt";
std::string pathToSettings = "/home/victor/calibration/orb_calib.yaml";

int main( int argc, char **argv )
{
    QStringList leftFileNames;
    QStringList rightFileNames;

    {
        QApplication a( argc, argv );

        QFile cssFile(":/resources/qss/style.css");

        if ( cssFile.open( QIODevice::ReadOnly ) ) {
            QString cssString( cssFile.readAll() );
            a.setStyleSheet( cssString );
        }
        else
            QMessageBox::critical( nullptr, "Error", "Can't load css file:" + cssFile.fileName() );

        StereoFilesListDialog dlg;

        if ( dlg.exec() == QDialog::Accepted ) {
            leftFileNames = dlg.leftFileNames();
            rightFileNames = dlg.rightFileNames();
        }

    }

    if ( leftFileNames.size() == rightFileNames.size() && leftFileNames.size() > 0 ) {
        ORB_SLAM2::System SLAM( pathToVocabulary, pathToSettings, ORB_SLAM2::System::STEREO, true );

        cv::Mat imLeft, imRight;

        double tframe = 0;

        for( int i = 0; i < leftFileNames.size(); ++i )
        {
            auto vstrImageLeft = leftFileNames[i].toStdString();
            auto vstrImageRight = rightFileNames[i].toStdString();

            imLeft = cv::imread( vstrImageLeft, cv::IMREAD_UNCHANGED );
            imRight = cv::imread( vstrImageRight, cv::IMREAD_UNCHANGED );
            tframe += 1.0/30.0;

            if ( imLeft.empty() ) {
                std::cerr << "Failed to load image: " << i << std::endl;
                return 1;
            }

            SLAM.TrackStereo(imLeft,imRight,tframe);
        }


        getchar();

        SLAM.Shutdown();

    }

    return 0;
}

