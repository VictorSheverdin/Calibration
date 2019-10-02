/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <iostream>

#include <src/viso2/viso_stereo.h>
#include <png++/png.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/viz.hpp"

#include "src/common/calibrationdatabase.h"

using namespace std;

int main (int argc, char** argv) {

    // sequence directory
  string dir = "/home/victor/datasets/Polygon";
  
    std::string pathToSettings = "/home/victor/calibration/calibration.yaml";

    StereoCalibrationDataShort calibrationData( pathToSettings );

    if ( calibrationData.isOk() ) {

        cv::namedWindow( "Features", cv::WINDOW_KEEPRATIO );
        cv::resizeWindow("Features", 800, 600);
        cv::moveWindow("Features", 80, 10);

        cv::viz::Viz3d vizWindow("Viz");
        cv::moveWindow("Viz", 500, 10);
        vizWindow.showWidget("coordSystemWidget", cv::viz::WCoordinateSystem());

        // set most important visual odometry parameters
        // for a full parameter list, look at: viso_stereo.h
        VisualOdometryStereo::parameters param;

        // calibration parameters for sequence 2010_03_09_drive_0019
        param.calib.f  = calibrationData.leftCameraResults().fx(); // focal length in pixels
        param.calib.cu = calibrationData.leftCameraResults().cx() - calibrationData.leftROI().x; // principal point (u-coordinate) in pixels
        param.calib.cv = calibrationData.leftCameraResults().cy() - calibrationData.leftROI().y; // principal point (v-coordinate) in pixels
        param.base     = calibrationData.distance(); // baseline in meters

        // init visual odometry
        VisualOdometryStereo viso(param);

        // current pose (this matrix transforms a point from the current
        // frame's camera coordinates to the first frame's camera coordinates)
        Matrix pose = Matrix::eye(4);

        std::vector<cv::Affine3d> trajectoryPoints;

      // loop through all frames i=0:372
      for (int32_t i=10000; i<20000; i++) {

        // input file names
        char base_name_left[256]; sprintf(base_name_left,"%05d_left.jpg",i);
        char base_name_right[256]; sprintf(base_name_right,"%05d_right.jpg",i);
        string left_img_file_name  = dir + "/left/" + base_name_left;
        string right_img_file_name = dir + "/right/" + base_name_right;

        std::cout << left_img_file_name << " "<<  right_img_file_name << std::endl;

        // catch image read/write errors here
        try {

            auto leftMat = cv::imread( left_img_file_name );
            auto rightMat = cv::imread( right_img_file_name );

            cv::Mat leftGray, rightGray;

            cv::cvtColor( leftMat, leftGray, cv::COLOR_BGR2GRAY );
            cv::cvtColor( rightMat, rightGray, cv::COLOR_BGR2GRAY );

            CvImage leftRectifiedImage;
            CvImage rightRectifiedImage;

            cv::remap( leftGray, leftRectifiedImage, calibrationData.leftRMap(), calibrationData.leftDMap(), cv::INTER_LANCZOS4 );
            cv::remap( rightGray, rightRectifiedImage, calibrationData.rightRMap(), calibrationData.rightDMap(), cv::INTER_LANCZOS4 );

            CvImage leftCroppedFrame;
            CvImage rightCroppedFrame;

            if (!calibrationData.leftROI().empty() && !calibrationData.rightROI().empty()) {
                leftCroppedFrame = leftRectifiedImage( calibrationData.leftROI() );
                rightCroppedFrame = rightRectifiedImage( calibrationData.leftROI() );
            }

            cv::imshow( "Features", leftCroppedFrame );

          // image dimensions
          int32_t width  = leftCroppedFrame.cols;
          int32_t height = leftCroppedFrame.rows;

          // convert input images to uint8_t buffer
          uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
          uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
          int32_t k=0;
          for (int32_t v=0; v<height; v++) {
            for (int32_t u=0; u<width; u++) {
              left_img_data[k]  = leftCroppedFrame.at<uchar>(u,v);
              right_img_data[k] = rightCroppedFrame.at<uchar>(u,v);
              k++;
            }
          }

          // status
          cout << "Processing: Frame: " << i;

          // compute visual odometry
          int32_t dims[] = { width, height, width };
          if (viso.process( left_img_data, right_img_data, dims ) ) {

            // on success, update current pose
            pose = pose * Matrix::inv( viso.getMotion() );

            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);

            for (int32_t i=0; i<3; i++)
              for (int32_t j=0; j<3; j++)
                  R.at<double>( i, j ) = pose.val[i][j];

            for (int32_t i=0; i<3; i++)
                t.at<double>( i ) = pose.val[i][3];

            cv::Affine3d cameraPose( R, t );
            trajectoryPoints.push_back(cameraPose);

            cv::viz::WTrajectory trajectory(trajectoryPoints);
            vizWindow.showWidget("trajectory", trajectory);

            cv::viz::WTrajectoryFrustums trajectoryFrustums( trajectoryPoints, cv::Vec2d( param.calib.f, param.calib.f ), 0.001 );
            vizWindow.showWidget("trajectoryFrustums", trajectoryFrustums);

            // output some statistics
            double num_matches = viso.getNumberOfMatches();
            double num_inliers = viso.getNumberOfInliers();
            cout << ", Matches: " << num_matches;
            cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
            cout << pose << endl << endl;

          } else {
            cout << " ... failed!" << endl;
          }

          // release uint8_t buffers
          free(left_img_data);
          free(right_img_data);

        // catch image read errors here
        } catch (...) {
          cerr << "ERROR: Couldn't read input files!" << endl;
          return 1;
        }

        vizWindow.spinOnce(15,  true);
        cv::waitKey(15);

      }

      // output
      cout << "Demo complete! Exiting ..." << endl;

    }

  // exit
  return 0;
}

