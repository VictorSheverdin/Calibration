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

    std::thread calcThread( [&] {

        for ( auto i = 23000; i < 25000; i++ ) {
            std::string leftFile = leftPath + std::to_string( i ) + "_left.jpg";
            std::string rightFile = rightPath + std::to_string( i ) + "_right.jpg";

            auto startTime = std::chrono::system_clock::now();
            slamSystem.track( leftFile, rightFile );
            std::cout << std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now() - startTime ).count() << std::endl;

        }

    } );

    calcThread.detach();

    while ( true ) {

        CvImage keyPointsImage;
        CvImage stereoPointsImage;
        CvImage tracksImage;
        std::list< std::shared_ptr< slam::WorldPoint > > worldPoints;
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

            std::vector< cv::Affine3d > leftTrajectoryPoints;
            std::vector< cv::Affine3d > rightTrajectoryPoints;

            for ( auto &i : frames ) {
                leftTrajectoryPoints.push_back( cv::Affine3d( i->leftFrame()->rotation().t(), cv::Mat( - i->leftFrame()->rotation().t() * i->leftFrame()->translation() ) ) );
                rightTrajectoryPoints.push_back( cv::Affine3d( i->rightFrame()->rotation().t(), cv::Mat( - i->rightFrame()->rotation().t() * i->rightFrame()->translation() ) ) );
            }

            cv::viz::WTrajectory leftTrajectory( leftTrajectoryPoints );
            vizWindow.showWidget( "leftTrajectory", leftTrajectory );
            cv::viz::WTrajectoryFrustums leftTrajectoryFrustums( leftTrajectoryPoints, cv::Vec2d( 1, 1 ), 0.5 );
            vizWindow.showWidget( "leftTrajectoryFrustums", leftTrajectoryFrustums );

            cv::viz::WTrajectory rightTrajectory( rightTrajectoryPoints );
            vizWindow.showWidget( "rightTrajectory", rightTrajectory );
            cv::viz::WTrajectoryFrustums rightTrajectoryFrustums( rightTrajectoryPoints, cv::Vec2d( 1, 1 ), 0.5 );
            vizWindow.showWidget( "rightTrajectoryFrustums", rightTrajectoryFrustums );

        }

        vizWindow.spinOnce();

        cv::waitKey( 1 );

    }

    return 0;

}
