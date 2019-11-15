#include "src/common/precompiled.h"

#include "system.h"

int main( int argc, char** argv )
{
    std::string path("/home/victor/Polygon/");
    std::string leftPath = path + "left/";
    std::string rightPath = path + "right/";

    slam::System slamSystem( path + "calibration.yaml" );

    for ( auto i = 9000; i < 15000; ++i ) {
        std::string leftFile = leftPath + std::to_string( i ) + "_left.jpg";
        std::string rightFile = rightPath + std::to_string( i ) + "_right.jpg";

        slamSystem.track( leftFile, rightFile );

    }

    return 0;
}
