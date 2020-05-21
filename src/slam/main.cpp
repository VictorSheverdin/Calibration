#include "src/common/precompiled.h"

#include "application.h"

#include <opencv2/viz.hpp>
#include <opencv2/core/ocl.hpp>

#include <QVTKOpenGLNativeWidget.h>

int main(int argc, char** argv)
{
    QSurfaceFormat::setDefaultFormat( QVTKOpenGLNativeWidget::defaultFormat() );

    Application a(argc, argv);

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

    return a.exec();

}

