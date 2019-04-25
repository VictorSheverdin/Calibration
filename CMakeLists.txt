cmake_minimum_required( VERSION 2.6 )
project ( Calibration )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set ( CALIBRATION_SOURCES
    "src/common/defs.h"
    "src/common/precompiled.h"
    "src/calibration/application.h"
    "src/calibration/application.cpp"
    "src/calibration/mainwindow.h"
    "src/calibration/mainwindow.cpp"
    "src/calibration/calibrationwidget.h"
    "src/calibration/calibrationwidget.cpp"
    "src/common/image.h"
    "src/common/image.cpp"
    "src/common/imagewidget.h"
    "src/common/imagewidget.cpp"
    "src/calibration/iconswidget.h"
    "src/calibration/iconswidget.cpp"
    "src/calibration/camerawidget.h"
    "src/calibration/camerawidget.cpp"
    "src/calibration/previewwidget.h"
    "src/calibration/previewwidget.cpp"
    "src/calibration/taskwidget.h"
    "src/calibration/taskwidget.cpp"
    "src/common/supportwidgets.h"
    "src/common/supportwidgets.cpp"
    "src/calibration/templateprocessor.h"
    "src/calibration/templateprocessor.cpp"
    "src/calibration/reportwidget.h"
    "src/calibration/reportwidget.cpp"
    "src/calibration/parameterswidget.h"
    "src/calibration/parameterswidget.cpp"
    "src/common/functions.h"
    "src/common/functions.cpp"
    "src/common/calibrationdata.h"
    "src/common/calibrationdata.cpp"
    "src/calibration/main.cpp"
)
set ( DISPARITY_SOURCES
    "src/common/defs.h"
    "src/common/precompiled.h"
    "src/disparity/application.h"
    "src/disparity/application.cpp"
    "src/disparity/mainwindow.h"
    "src/disparity/mainwindow.cpp"
    "src/common/image.h"
    "src/common/image.cpp"
    "src/common/imagewidget.h"
    "src/common/imagewidget.cpp"
    "src/common/calibrationdata.h"
    "src/common/calibrationdata.cpp"
    "src/disparity/disparitycontrolwidget.h"
    "src/disparity/disparitycontrolwidget.cpp"
    "src/disparity/disparitypreviewwidget.h"
    "src/disparity/disparitypreviewwidget.cpp"
    "src/common/functions.h"
    "src/common/functions.cpp"
    "src/common/supportwidgets.h"
    "src/common/supportwidgets.cpp"
    "src/disparity/pclwidget.h"
    "src/disparity/pclwidget.cpp"
    "src/disparity/main.cpp"
)

set ( RESOURCES
    "resources/resources.qrc"
)

find_package(Qt5Widgets REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_definitions( ${QT_DEFINITIONS} )
include_directories( ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

QT5_ADD_RESOURCES( RES_SOURCES ${RESOURCES} )

add_executable( calibration ${CALIBRATION_SOURCES} ${RES_SOURCES} )
add_executable( disparity ${DISPARITY_SOURCES} ${RES_SOURCES} )

target_include_directories( calibration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/include )
target_include_directories( disparity PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/include )

target_link_libraries( calibration ${Qt5Widgets_LIBRARIES} ${Qt5Xml_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${YAML_CPP_LIBRARIES} )
target_link_libraries( disparity ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${PCL_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/src/vimba/lib/libVimbaCPP.so ${YAML_CPP_LIBRARIES} )
