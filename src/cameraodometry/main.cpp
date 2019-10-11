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

#include "src/common/functions.h"
#include "src/common/vimbacamera.h"

using namespace std;

std::string pathToVocabulary = "/home/victor/calibration/ORBvoc.txt";
std::string pathToSettings = "/home/victor/calibration/calibration.yaml";

int main( int , char ** )
{   
    checkVimbaStatus( AVT::VmbAPI::VimbaSystem::GetInstance().Startup(), "Could not start Vimba system" );

    StereoCamera camera( "192.168.80.66", "192.168.80.82" );

    StereoCalibrationDataShort calibrationData( pathToSettings );
    ORB_SLAM2::Settings settings( calibrationData );

    ORB_SLAM2::System SLAM( pathToVocabulary, settings, ORB_SLAM2::System::STEREO, true );

    auto prevTime = std::chrono::system_clock::now();

    while( true )
    {
        // camera.updateFrame();
        auto frame = camera.getFrame();

        if ( !frame.empty() ) {
            auto imLeft = frame.leftFrame();
            auto imRight = frame.rightFrame();

            auto currentTime = std::chrono::system_clock::now();

            auto time = std::chrono::duration_cast< std::chrono::milliseconds >( currentTime - prevTime ).count();
            prevTime = currentTime;

            SLAM.TrackStereo( imLeft, imRight, static_cast<double>( time / 1000.0 ) );

        }

    }


    getchar();

    SLAM.Shutdown();

    AVT::VmbAPI::VimbaSystem::GetInstance().Shutdown();

    return 0;
}

