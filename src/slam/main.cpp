#include "src/common/precompiled.h"

#include "system.h"
#include "worldpoint.h"
#include "frame.h"

#include <opencv2/viz.hpp>

#include <thread>

int main( int argc, char** argv )
{
    std::string path("/home/victor/Polygon/");
    std::string leftPath = path + "left/";
    std::string rightPath = path + "right/";

    cv::namedWindow( "KeyPoints", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "KeyPoints", 800, 600 );
    cv::moveWindow( "KeyPoints", 80, 10 );

    cv::namedWindow( "Stereo", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "Stereo", 800, 600 );
    cv::moveWindow( "Stereo", 880, 10 );

    cv::namedWindow( "Track", cv::WINDOW_KEEPRATIO );
    cv::resizeWindow( "Track", 800, 600 );
    cv::moveWindow( "Track", 80, 900 );

    cv::viz::Viz3d vizWindow( "Viz3d" );
    vizWindow.showWidget( "coordSystemWidget", cv::viz::WCoordinateSystem() );
    vizWindow.setWindowPosition( cv::Point( 880, 900 ) );
    vizWindow.setWindowSize( cv::Size( 800, 600 ) );

    slam::System slamSystem( path + "calibration.yaml" );

    std::mutex systemMutex;

    std::thread calcThread( [&] {


        for ( auto i = 10000; i < 15000; i++ ) {
            std::string leftFile = leftPath + std::to_string( i ) + "_left.jpg";
            std::string rightFile = rightPath + std::to_string( i ) + "_right.jpg";

            slamSystem.track( leftFile, rightFile );

        }

    } );

    calcThread.detach();

    while ( true ) {

        CvImage keyPointsImage;
        CvImage stereoPointsImage;
        CvImage tracksImage;
        std::vector< std::shared_ptr< slam::WorldPoint > > worldPoints;
        std::list< std::shared_ptr< slam::StereoFrame > > frames;

        keyPointsImage = slamSystem.keyPointsImage();
        stereoPointsImage = slamSystem.stereoPointsImage();
        tracksImage = slamSystem.tracksImage();
        worldPoints = slamSystem.worldPoints();
        frames = slamSystem.frames();

        if ( !keyPointsImage.empty() )
            cv::imshow( "KeyPoints", keyPointsImage );

        if ( !stereoPointsImage.empty() )
            cv::imshow( "Stereo", stereoPointsImage );

        if ( !tracksImage.empty() )
            cv::imshow( "Track", tracksImage );

        std::vector< cv::Vec3d > points;
        std::vector< cv::Vec4b > colors;

        for ( auto &i : worldPoints ) {
            points.push_back( i->point() );
            colors.push_back( i->color() );
        }

        if ( !points.empty() ) {
            cv::viz::WCloud cloud( points, colors );
            cloud.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
            vizWindow.showWidget( "Point cloud", cloud );

            std::vector< cv::Affine3d > trajectoryPoints;

            for ( auto &i : frames ) {
                cv::Affine3d cameraPose( i->leftFrame()->rotation(), i->leftFrame()->translation() );
                trajectoryPoints.push_back( cameraPose );
            }

            cv::viz::WTrajectory trajectory( trajectoryPoints );
            vizWindow.showWidget( "trajectory", trajectory );

            cv::viz::WTrajectoryFrustums trajectoryFrustums( trajectoryPoints, cv::Vec2d( 1, 1 ), 1 );
            vizWindow.showWidget( "trajectoryFrustums", trajectoryFrustums );

        }

        vizWindow.spinOnce();

        cv::waitKey( 1 );

    }

    return 0;

}
