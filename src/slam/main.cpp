#include "src/common/precompiled.h"

#include "src/common/rectificationprocessor.h"

#include "world.h"
#include "mappoint.h"
#include "frame.h"

#include <opencv2/viz.hpp>

#include <thread>

#include <opencv2/core/ocl.hpp>

int main( int, char** )
{
    cv::ocl::Context context;
    std::vector< cv::ocl::PlatformInfo > platforms;

   cv::ocl::getPlatfomsInfo(platforms);

    for (size_t i = 0; i < platforms.size(); i++)
    {
        const cv::ocl::PlatformInfo* platform = &platforms[i];
        std::cout << "Platform Name: " << platform->name() << "\n" << std::endl;
        cv::ocl::Device current_device;

        for (int j = 0; j < platform->deviceNumber(); j++)
        {
            platform->getDevice(current_device, j);
            int deviceType = current_device.type();
            std::cout << "Device name:  " << current_device.name() << std::endl;
            if (deviceType == 2)
                std::cout << context.ndevices() << " CPU devices are detected." << std::endl;
            if (deviceType == 4)
                std::cout << context.ndevices() << " GPU devices are detected." << std::endl;
        }

    }

    cv::ocl::setUseOpenCL( true );

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
    vizWindow.setWindowPosition( cv::Point( 880, 900 ) );
    vizWindow.setWindowSize( cv::Size( 800, 600 ) );

    vizWindow.showWidget( "coordSystemWidget", cv::viz::WCoordinateSystem() );

    StereoCalibrationDataShort calibration( path + "calibration.yaml" );

    auto system = slam::World::create( calibration.leftProjectionMatrix(), calibration.rightProjectionMatrix() );
    system->createMap();

    auto scaleFactor = 1.0;

    auto cropRect = calibration.cropRect();
    auto principal = cv::Vec2f( -cropRect.x, -cropRect.y );

    system->movePrincipalPoint( principal );

    system->multiplicateCameraMatrix( scaleFactor );

    std::thread calcThread( [ & ] {

        StereoRectificationProcessor rectificationProcessor( path + "calibration.yaml" );

        for ( auto i = /*8170*/5900; i < 30000; i++ ) {

            std::cout << "Processing frame " << i << std::endl;
            std::string leftFile = leftPath + std::to_string( i ) + "_left.jpg";
            std::string rightFile = rightPath + std::to_string( i ) + "_right.jpg";

            CvImage leftImage( leftFile );
            CvImage rightImage( rightFile );

            CvImage leftRectifiedImage;
            CvImage rightRectifiedImage;

            CvImage leftCroppedImage;
            CvImage rightCroppedImage;

            if ( rectificationProcessor.rectify( leftImage, rightImage, &leftRectifiedImage, &rightRectifiedImage )
                        && rectificationProcessor.crop( leftRectifiedImage, rightRectifiedImage, &leftCroppedImage, &rightCroppedImage ) ) {

                CvImage leftResizedImage;
                CvImage rightResizedImage;

                cv::resize( leftCroppedImage, leftResizedImage, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC );
                cv::resize( rightCroppedImage, rightResizedImage, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC );

                system->track( leftResizedImage, rightResizedImage );

            }

            // std::this_thread::sleep_for( std::chrono::seconds( 3 ) );

        }

    } );

    calcThread.detach();

    while ( true ) {

        auto keyPointsImage = system->keyPointsImage();
        auto stereoPointsImage = system->stereoPointsImage();
        auto tracksImage = system->tracksImage();
        auto mapPoints = system->mapPoints();
        auto frames = system->frames();

        if ( !keyPointsImage.empty() )
            cv::imshow( "KeyPoints", keyPointsImage );

        if ( !stereoPointsImage.empty() )
            cv::imshow( "Stereo", stereoPointsImage );

        if ( !tracksImage.empty() )
            cv::imshow( "Track", tracksImage );

        std::vector< cv::Vec3d > points;
        std::vector< cv::Vec4b > colors;

        for ( auto &i : mapPoints ) {

            if ( i ) {

                auto point = i->point();

                points.push_back( point );
                colors.push_back( i->color() );

            }

        }

        if ( !points.empty() ) {

            cv::viz::WCloud cloud( points, colors );
            cloud.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
            vizWindow.showWidget( "Point cloud", cloud );

            std::vector< cv::Affine3d > leftTrajectoryPoints;
            std::vector< cv::Affine3d > rightTrajectoryPoints;

            for ( auto &i : frames ) {

                auto stereoFrame = std::dynamic_pointer_cast< slam::StereoFrame >( i );

                if ( stereoFrame ) {

                    leftTrajectoryPoints.push_back( cv::Affine3d( stereoFrame->leftFrame()->rotation().t(), cv::Mat( - stereoFrame->leftFrame()->rotation().t() * stereoFrame->leftFrame()->translation() ) ) );
                    rightTrajectoryPoints.push_back( cv::Affine3d( stereoFrame->rightFrame()->rotation().t(), cv::Mat( - stereoFrame->rightFrame()->rotation().t() * stereoFrame->rightFrame()->translation() ) ) );

                }
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

        vizWindow.spinOnce( 150 );

        cv::waitKey( 1 );

    }

    return 0;

}
