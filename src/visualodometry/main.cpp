#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/sfm.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>

#include "src/common/fileslistwidget.h"
#include <QApplication>
#include <QFile>
#include <QMessageBox>
#include <QDebug>

using namespace cv;
using namespace std;

TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);

void featureTracking(Mat sourceFrame, Mat targetFrame, vector< KeyPoint >& points1, vector< KeyPoint >& points2 )
{
    cv::Mat gray1;
    cv::cvtColor(sourceFrame, gray1, COLOR_BGR2GRAY);

    cv::Mat gray2;
    cv::cvtColor(targetFrame, gray2, COLOR_BGR2GRAY);

    static auto detector = cv::AgastFeatureDetector::create( 50 );
    static auto descriptor = cv::xfeatures2d::DAISY::create();

    std::vector< cv::KeyPoint > sourceKeypoints;
    detector->detect( gray1, sourceKeypoints );

    cv::Mat sourceDescriptors;
    descriptor->compute( gray1, sourceKeypoints, sourceDescriptors );

    std::vector< cv::KeyPoint > targetKeypoints;
    cv::Mat targetDescriptors;
    detector->detect( gray2, targetKeypoints );
    descriptor->compute( gray2, targetKeypoints, targetDescriptors );

    static auto matcher = cv::FlannBasedMatcher::create();

    std::vector< std::vector< DMatch > > matches;
    matcher->knnMatch( sourceDescriptors, targetDescriptors, matches, 2 );

    points1.clear();
    points2.clear();

    for ( size_t i = 0; i < matches.size(); ++i ) {
        if ( matches[i][0].distance < 2.0/3.0 * matches[i][1].distance ) {
            points1.push_back( sourceKeypoints[ matches[i][0].queryIdx ] );
            points2.push_back( targetKeypoints[ matches[i][0].trainIdx ] );

        }

    }


    vector<Point2f> trackingPoints;
    vector<Point2f> resultPoints;

    cv::goodFeaturesToTrack( gray1, trackingPoints, 5000, 0.1, 10, cv::noArray(), 3, true, 0.04 );

    vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK( gray1, gray2, trackingPoints, resultPoints, status, err );

    points1.clear();
    points2.clear();

    if ( !status.empty() && !err.empty() ) {

        float minErr = err.front();
        float maxErr = err.front();

        for ( auto i = 1; i < err.size(); ++i ) {
            minErr = min( minErr, err[i] );
            maxErr = max( maxErr, err[i] );
        }

        for (auto i = 0; i < status.size(); ++i ) {
            if ( status[i] && err[i] < minErr + 0.5 * ( maxErr - minErr ) ) {
                points1.push_back( cv::KeyPoint( trackingPoints[i], 0 ) );
                points2.push_back( cv::KeyPoint( resultPoints[i], 0 ) );
            }

        }

    }

}

void drawPoints( Mat frame, vector<KeyPoint>& points, const cv::Scalar &color = cv::Scalar(0, 0, 255, 255) )
{
    for (auto &i : points)
        cv::circle(frame, i.pt, 5, color, -1);

}

void drawLines(Mat frame, vector<KeyPoint>& points1, vector<KeyPoint>& points2)
{
    for ( size_t i = 0; i <points1.size(); ++i )
        cv::line(frame, points1[i].pt, points2[i].pt, cv::Scalar(0, 255, 0, 255));

}

int main( int argc, char** argv )
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

        namedWindow( "Features", WINDOW_KEEPRATIO );
        cv::resizeWindow("Features", 800, 600);
        cv::moveWindow("Features", 80, 10);

        cv:viz::Viz3d vizWindow("Viz");
        cv::moveWindow("Viz", 500, 10);
        vizWindow.showWidget("coordSystemWidget", cv::viz::WCoordinateSystem());


        Mat camMatrix = ( Mat_<double>(3,3) << 9.1073236417442979e+02, 0., 1.0011878386845302e+03, 0.,
                         9.0913115226566401e+02, 1.0080606237570406e+03, 0., 0., 1. );

        std::vector<double> distCoefficients = { 2.5205656544025762e-02, -1.9577404877149228e-02,
                                                 -8.4204471895308317e-04, -2.6845992072923154e-03, 0. };

        std::vector<cv::Affine3d> trajectoryPoints;

        cv::Mat captureFrame;
        cv::Mat prevFrame;
        vector<cv::KeyPoint> prevFeaturePoints;
        cv::Mat curFrame;
        vector<cv::KeyPoint> curFeaturePoints;

        int i = 10000;

        cv::Mat rTotal = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat tTotal = cv::Mat::zeros(3, 1, CV_64F);

        std::vector< cv::Scalar > tempColors;

        std::vector<cv::Point3f> points;
        std::vector<cv::Vec4b> colors;

        do {

            auto vstrImageLeft = leftDir + "/" + QString::number(i) + "_left.jpg";
            auto vstrImageRight = rightDir + "/" + QString::number(i) + "_right.jpg";

            qDebug() << vstrImageLeft << vstrImageRight;

            captureFrame = imread( vstrImageLeft.toStdString() );

            if (!captureFrame.empty()) {

                cv::Mat frame;

                cv::undistort(captureFrame, frame, camMatrix, distCoefficients);

                if ( !prevFrame.empty() ) {
                    frame.copyTo(curFrame);
                    featureTracking(prevFrame, curFrame, prevFeaturePoints, curFeaturePoints );

                    if ( prevFeaturePoints.size() > 8 ) {

                        Mat R, t;
                        cv::Mat f;
                        cv::Mat e;

                        vector<cv::Point2f> prevPoints;
                        vector<cv::Point2f> curPoints;

                        for ( auto &i : prevFeaturePoints )
                            prevPoints.push_back( i.pt );

                        for ( auto &i : curFeaturePoints )
                            curPoints.push_back( i.pt );

                        f = cv::findFundamentalMat( prevPoints, curPoints, cv::noArray() );
                        e = /*cv::findEssentialMat( prevFeaturePoints, curFeaturePoints, camMatrix )*/
                                camMatrix.t() * f * camMatrix;

                        vector<cv::KeyPoint> prevRefinedPoints = prevFeaturePoints;
                        vector<cv::KeyPoint> curRefinedPoints = curFeaturePoints;

                        /*prevRefinedPoints.clear();
                        curRefinedPoints.clear();
                        for ( size_t i = 0; i < prevFeaturePoints.size(); ++i ) {
                            cv::Mat p1( 1, 3, CV_64FC1 );
                            p1.at<double>( 0, 0 ) = prevFeaturePoints[i].pt.x;
                            p1.at<double>( 0, 1 ) = prevFeaturePoints[i].pt.y;
                            p1.at<double>( 0, 2 ) = 1.0;

                            cv::Mat p2( 3, 1, CV_64FC1 );
                            p2.at<double>( 0, 0 ) = curFeaturePoints[i].pt.x;
                            p2.at<double>( 1, 0 ) = curFeaturePoints[i].pt.y;
                            p2.at<double>( 2, 0 ) = 1.0;

                            cv::Mat prod = p1 * f * p2;

                            if ( fabs( prod.at< double >( 0, 0 ) ) < 0.5 ) {
                                prevRefinedPoints.push_back( prevFeaturePoints[i] );
                                curRefinedPoints.push_back( curFeaturePoints[i] );
                            }

                            std:: cout << prod.at< double >( 0, 0 ) << " ";

                        }*/

                        if ( prevRefinedPoints.size() > 8 ) {

                            cv::Mat renderFrame;
                            curFrame.copyTo(renderFrame);
                            drawPoints( renderFrame, curRefinedPoints );
                            drawLines( renderFrame, prevRefinedPoints, curRefinedPoints );

                            cv::imshow("Features", renderFrame);

                            cv::Mat prevPts(2, prevRefinedPoints.size(), CV_64F);
                            cv::Mat curPts(2, curRefinedPoints.size(), CV_64F);

                            for (auto i = 0; i < prevRefinedPoints.size(); ++i) {
                                prevPts.at<double>(0, i) = prevRefinedPoints[i].pt.x;
                                prevPts.at<double>(1, i) = prevRefinedPoints[i].pt.y;
                            }

                            for (auto i = 0; i < curRefinedPoints.size(); ++i) {
                                curPts.at<double>(0, i) = curRefinedPoints[i].pt.x;
                                curPts.at<double>(1, i) = curRefinedPoints[i].pt.y;
                            }

                            std::vector<cv::Mat> rvec;
                            std::vector<cv::Mat> tvec;

                            cv::sfm::motionFromEssential(e, rvec, tvec);

                            cv::Mat pt1 = (Mat_<double>(2,1) << prevRefinedPoints.front().pt.x, prevRefinedPoints.front().pt.y);
                            cv::Mat pt2 = (Mat_<double>(2,1) << curRefinedPoints.front().pt.x, curRefinedPoints.front().pt.y);

                            int id = cv::sfm::motionFromEssentialChooseSolution(rvec, tvec, camMatrix, pt1, camMatrix, pt2);

                            if ( id > 0 && id < 4 ) {

                                R = rvec[id];
                                t = tvec[id];

                                cv::Mat p1, p2;

                                cv::sfm::projectionFromKRt(camMatrix, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), p1);
                                cv::sfm::projectionFromKRt(camMatrix, R, t, p2);

                                tTotal = tTotal + rTotal * t;
                                rTotal = rTotal * R;

                                std::vector<cv::Mat> Ps;
                                Ps.push_back(p1);
                                Ps.push_back(p2);

                                std::vector<Mat_<double> > pts(2);
                                pts[0].create(2, prevRefinedPoints.size());
                                pts[1].create(2, curRefinedPoints.size());
                                for (auto i = 0; i < prevRefinedPoints.size(); ++i) {
                                    pts[0].row(0).col(i) = prevRefinedPoints[i].pt.x;
                                    pts[0].row(1).col(i) = prevRefinedPoints[i].pt.y;
                                }
                                for (auto i = 0; i < curRefinedPoints.size(); ++i) {
                                    pts[1].row(0).col(i) = curRefinedPoints[i].pt.x;
                                    pts[1].row(1).col(i) = curRefinedPoints[i].pt.y;
                                }

                                cv::Mat points3d;

                                cv::sfm::triangulatePoints(pts, Ps, points3d);

                                points.clear();
                                colors.clear();

                                for (auto i = 0; i <points3d.cols; ++i) {

                                    auto x = points3d.at<double>(0, i);
                                    auto y = -points3d.at<double>(1, i);
                                    auto z = -points3d.at<double>(2, i);

                                    if (fabs(x) < 1e2 && fabs(y) < 1e2 && fabs(z) < 1e2) {
                                        cv::Mat pointVector = (Mat_<double>(3,1) << x, y, z);
                                        pointVector = rTotal * pointVector + tTotal;
                                        cv::Point3f point(pointVector.at<double>(0),
                                                          pointVector.at<double>(1),
                                                          pointVector.at<double>(2));
                                        points.push_back(point);
                                        colors.push_back( curFrame.at< cv::Vec4b >( curRefinedPoints[i].pt.x, curRefinedPoints[i].pt.y ) );

                                    }

                                }

                                cv::Affine3d cameraPose(rTotal, tTotal);
                                trajectoryPoints.push_back(cameraPose);

                                cv::viz::WTrajectory trajectory(trajectoryPoints);
                                vizWindow.showWidget("trajectory", trajectory);

                                cv::viz::WTrajectoryFrustums trajectoryFrustums( trajectoryPoints, cv::Vec2d( camMatrix.at<double>( 0, 0 ), camMatrix.at<double>( 1, 1 ) ), 0.001 );
                                vizWindow.showWidget("trajectoryFrustums", trajectoryFrustums);

                                if (!points.empty()) {
                                    cv::viz::WCloud cloud( points, colors );
                                    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 1);
                                    cloud.setColor(cv::viz::Color(0, 255, 255));
                                    vizWindow.showWidget("Point cloud", cloud);

                                }

                            }

                        }

                    }
                    else
                        std::cout << "empty" << std::endl;

               }

                frame.copyTo(prevFrame);

                vizWindow.spinOnce(15,  true);
                cv::waitKey(15);
                i+=1;

            }

        } while (!captureFrame.empty());

        vizWindow.spin();

    }


    return 0;
}

